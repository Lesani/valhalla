#ifndef VALHALLA_SIF_SCENIC_COST_HELPERS_H_
#define VALHALLA_SIF_SCENIC_COST_HELPERS_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace sif {

// Maximum extra cost factor a dead-straight edge accumulates relative to a
// maximally curvy edge of the same class (the "detour budget"): at alpha=1.0
// a dead-straight edge costs (1 + kCurvyDetourCap)x its base cost.
inline constexpr float kCurvyDetourCap = 1.5f;

// v3: the per-edge byte now carries CURVE DENSITY * 100 (Menger radius-binning),
// not the legacy 0..255 sinuosity. An edge that is entirely within a curve has
// density ~1.0 (byte ~100); tight hairpins reach ~2.0 (byte ~200). So "fully
// curvy, zero penalty" is reached at byte kCurveDensityFullByte, not 255. Tune
// against the rebuilt tiles.
inline constexpr float kCurveDensityFullByte = 120.0f;

/**
 * Multiplicative cost PENALTY for straight edges (better_mc_routing v1.1,
 * Issue #18 — admissible cost model).
 *
 * v1 used a sub-unit discount (`curvy_bonus` in [1-alpha, 1.0]) which made
 * EdgeCost(motorcycle_curvy) < EdgeCost(motorcycle) on curvy edges. Cost
 * factors below 1.0 break the admissibility of A*'s heuristic (calibrated
 * against base costs) and produce inconsistent expansion orders. v1.1
 * inverts the model: the curviest edge pays exactly its base cost and
 * everything straighter pays MORE, so every factor is >= 1.0.
 *
 * Returns a factor in [1.0, 1 + alpha * k]:
 *   - sinuosity_byte = 255 (max curvy) -> 1.0           (base cost)
 *   - sinuosity_byte = 0   (straight)  -> 1 + alpha * k (full penalty)
 *   - linearly interpolated in between
 *
 * alpha = 0 disables the penalty regardless of sinuosity. alpha is expected
 * to come from the caller already clamped to [0.0, 0.95] (Issue 06's
 * ParseMotorcycleCurvyCostOptions does this); the function still produces
 * sane (>= 1.0) output for alpha in [0, 1] and k >= 0.
 *
 * Pure function — no globals, no allocations — directly unit-testable.
 */
inline float straightness_penalty(uint8_t sinuosity_byte, float alpha, float k = kCurvyDetourCap) {
  // v3: normalize the curve-density byte by kCurveDensityFullByte (not 255), so
  // a genuinely curvy edge reaches s=1.0 (no penalty) and only straighter edges
  // pay more. (Var/param name kept as `sinuosity_byte` for minimal churn.)
  const float s = std::min(1.0f, sinuosity_byte / kCurveDensityFullByte);
  return 1.0f + alpha * k * (1.0f - s); // curviest edge = base cost; dead straight = (1+αk)×
}

/**
 * Per road-class cost multipliers for motorcycle_curvy (better_mc_routing v1).
 * Issue 07 — the Calimoto fix lives in the `residential` value: a high cost
 * weight there discourages routing through village residential streets even
 * when they happen to be slightly curvier than the main valley road.
 *
 * v1.1 (Issue #18): renormalized so the cheapest class (tertiary) is exactly
 * 1.0 — the old table had tertiary 0.85 and unclassified 0.90, i.e. sub-unit
 * factors that violated admissibility. The new values are the old table
 * divided by 0.85; relative preferences between classes are unchanged.
 *
 * Values are compile-time defaults; not exposed in the v1 API surface.
 * `living_street`, `service`, `track` are Use enum sub-categories that
 * the helper checks BEFORE classification() — they override the RoadClass.
 */
struct ClassMultipliers {
  float motorway = 5.88f;
  float trunk = 3.53f;
  float primary = 1.76f;
  float secondary = 1.18f;
  float tertiary = 1.00f; // baseline — cheapest class by design
  float unclassified = 1.06f;
  float residential = 1.65f; // Calimoto fix
  float living_street = 2.35f;
  float service = 2.94f;
  float track = 5.88f;
};

// Profile-rework D1: use_highways / use_trails scale three class-multiplier
// rows. These are PER-REQUEST constants — compute the scaled table ONCE at
// costing construction (MotorcycleCurvyCost ctor), never per edge. All rows
// stay >= 1.0 (admissibility invariant, Issue #18); the formulas guarantee it.
inline constexpr float kMotorwayMultMin = 1.15f;      // use_highways = 1.0
inline constexpr float kMotorwayMultMax = 8.0f;       // use_highways = 0.0
inline constexpr float kTrunkMultMin = 1.10f;         // use_highways = 1.0
inline constexpr float kTrunkMultMax = 4.5f;          // use_highways = 0.0
inline constexpr float kHighwayMultExponent = 1.5f;   // (1 - uh) ^ 1.5 easing
inline constexpr float kTrackMultAtZero = 6.0f;       // use_trails = 0.0 (~ today's 5.88)
inline constexpr float kTrackMultSlope = 5.0f;        // track(ut) = 6.0 - 5.0*ut -> ut=1 -> 1.0

// motorway(uh) = 1.15 + (8.0 - 1.15)*(1-uh)^1.5 ; trunk(uh) = 1.10 + (4.5-1.10)*(1-uh)^1.5
inline float highway_class_multiplier(float lo, float hi, float use_highways) {
  const float uh = std::clamp(use_highways, 0.0f, 1.0f);
  return lo + (hi - lo) * std::pow(1.0f - uh, kHighwayMultExponent);
}

// track(ut) = 6.0 - 5.0*ut ; floored at 1.0 to hold the >= 1.0 invariant.
inline float track_class_multiplier(float use_trails) {
  const float ut = std::clamp(use_trails, 0.0f, 1.0f);
  return std::max(1.0f, kTrackMultAtZero - kTrackMultSlope * ut);
}

// D2 (profile character): use_small_roads scales one small-road row toward 1.0.
// row(usr) = 1.0 + (row_default - 1.0) * (1.0 - usr). usr=0 keeps the compile
// default (Calimoto fix intact for every other profile); usr=1 -> 1.0. Since
// every small-road row_default >= 1.0, the result stays >= 1.0 for usr in [0,1]
// (admissibility invariant, Issue #18).
inline float small_road_multiplier(float row_default, float use_small_roads) {
  const float usr = std::clamp(use_small_roads, 0.0f, 1.0f);
  return 1.0f + (row_default - 1.0f) * (1.0f - usr);
}

// Build the per-request table from the parsed stock options. Every row except
// motorway/trunk/track and the four small-road rows keeps the compile-time
// ClassMultipliers default. use_small_roads defaults to 0.0f so existing 2-arg
// callers (and gtests) keep today's table unchanged.
inline ClassMultipliers scaled_class_multipliers(float use_highways,
                                                 float use_trails,
                                                 float use_small_roads = 0.0f) {
  ClassMultipliers w; // compile-time defaults (primary/secondary/tertiary/... untouched)
  w.motorway = highway_class_multiplier(kMotorwayMultMin, kMotorwayMultMax, use_highways);
  w.trunk = highway_class_multiplier(kTrunkMultMin, kTrunkMultMax, use_highways);
  w.track = track_class_multiplier(use_trails);
  // D2: small-road rows scale toward 1.0 (usr default 0 == today's table).
  w.residential = small_road_multiplier(w.residential, use_small_roads);
  w.living_street = small_road_multiplier(w.living_street, use_small_roads);
  w.service = small_road_multiplier(w.service, use_small_roads);
  w.unclassified = small_road_multiplier(w.unclassified, use_small_roads);
  return w;
}

/**
 * Pure lookup for the cost multiplier of an edge with the given RoadClass
 * and Use. Use sub-categories (kTrack, kLivingStreet) win over RoadClass
 * — taking these from the more specific Use enum is what lets the helper
 * distinguish "residential road" from "living street" (both are
 * kResidential / kServiceOther at the RoadClass level).
 *
 * Pure function — no globals, no allocations.
 */
inline float class_multiplier(baldr::RoadClass cls,
                              baldr::Use use,
                              const ClassMultipliers& w) {
  using baldr::Use;
  if (use == Use::kTrack) {
    return w.track;
  }
  if (use == Use::kLivingStreet) {
    return w.living_street;
  }
  using baldr::RoadClass;
  switch (cls) {
    case RoadClass::kMotorway:
      return w.motorway;
    case RoadClass::kTrunk:
      return w.trunk;
    case RoadClass::kPrimary:
      return w.primary;
    case RoadClass::kSecondary:
      return w.secondary;
    case RoadClass::kTertiary:
      return w.tertiary;
    case RoadClass::kUnclassified:
      return w.unclassified;
    case RoadClass::kResidential:
      return w.residential;
    case RoadClass::kServiceOther:
      return w.service;
    default:
      return 1.0f;
  }
}

/**
 * Class-based heuristic: a tolled edge is "scenic" (a famous mountain pass,
 * scenic byway) if it's on secondary/tertiary/unclassified road; otherwise
 * it's a "road toll" (highway tolls, tunnels). Issue 08.
 *
 * PRD spot-checked this against 5 real Austrian tolled roads; 95% accurate
 * without requiring spatial inference from OSM `mountain_pass=yes` nodes,
 * which is deferred to v2.
 */
inline bool is_scenic_toll(bool has_toll, baldr::RoadClass cls) {
  if (!has_toll) {
    return false;
  }
  using baldr::RoadClass;
  return cls == RoadClass::kSecondary || cls == RoadClass::kTertiary ||
         cls == RoadClass::kUnclassified;
}

/**
 * Cost multiplier for a tolled edge under MotorcycleCurvyCost (Issue 08;
 * remapped into [1.0, 1.6] by Issue #18 — admissible cost model).
 *
 * Non-tolled edges return 1.0 (no effect).
 * Scenic tolls scale with the user-tunable use_scenic_tolls option:
 *   use_scenic_tolls = 0.7 -> 1.0  (full appetite: toll is cost-NEUTRAL)
 *   use_scenic_tolls = 0.5 -> 1.24 (default: mild avoid)
 *   use_scenic_tolls = 0.2 -> 1.6  (avoid)
 * Road tolls (motorway/trunk/primary) are FIXED at 1.6 regardless of the
 * user's scenic-toll setting — the rider can't accidentally unlock the
 * A10 motorway tunnel by maxing out scenic-pass appetite.
 *
 * SEMANTIC SHIFT (v1.1): in v1 a scenic toll at max appetite was a 0.6x
 * DISCOUNT — a tolled mountain pass could beat an equally curvy un-tolled
 * road. Sub-unit multipliers break A* admissibility, so v1.1 caps the best
 * case at 1.0: a scenic toll now tops out at NEUTRAL vs un-tolled roads.
 * That is the price of admissibility — scenic tolls can no longer be
 * actively attracted, only "not punished".
 *
 * Caller must pre-clamp use_scenic_tolls to [0.2, 0.7]; this function
 * still produces sane output for slightly out-of-range values but the
 * range is the contract (and the >= 1.0 invariant only holds inside it).
 */
inline float toll_multiplier(bool has_toll,
                             baldr::RoadClass cls,
                             float use_scenic_tolls) {
  if (!has_toll) {
    return 1.0f;
  }
  if (is_scenic_toll(has_toll, cls)) {
    // Linear map [0.2, 0.7] -> [1.6, 1.0] (ust pre-clamped to [0.2, 0.7]).
    return 1.6f - 1.2f * (use_scenic_tolls - 0.2f);
  }
  // Road toll: fixed hard avoid (= use_scenic_tolls of 0.2).
  return 1.6f;
}

// D1 rev.2 (profile character): paved-surface penalty for MotorcycleCurvyCost
// only. Above use_trails 0.5 each PAVED edge is penalized proportionally to
// its assigned speed so its per-km cost equals a kUnpavedRefSpeed unpaved
// edge's, times a constant asphalt aversion: route choice becomes DISTANCE-
// driven with asphalt at a fixed per-km premium (owner: "time is not a
// relevant factor for adventure"). ut <= 0.5 is inert; unpaved edges
// (Surface >= kCompacted == 3) are never penalized. Always >= 1.0.
inline constexpr float kUnpavedRefSpeed = 25.0f; // measured corridor gravel speed
inline constexpr float kPavedAversion = 2.5f;    // asphalt per-km premium at ut=1.0

inline float paved_multiplier(baldr::Surface surface, float use_trails, float edge_speed_kph) {
  const float ut = std::clamp(use_trails, 0.0f, 1.0f);
  if (ut <= 0.5f || static_cast<uint8_t>(surface) >= 3) {
    return 1.0f;
  }
  const float t = (ut - 0.5f) * 2.0f; // (0, 1]
  const float pm_full = std::max(1.0f, edge_speed_kph / kUnpavedRefSpeed) * kPavedAversion;
  return 1.0f + (pm_full - 1.0f) * t;
}

// Patch 0024 (#192): a curvy route must not leave a motorway/trunk by the
// off-ramp only to rejoin it by the next on-ramp. The detour is class-
// multiplier arbitrage (exit ramps are often classified `primary`, 1.76x,
// while the mainline is `motorway`, up to 8x), not curvature, and the stock
// ramp transition is only kTCRamp = 1.5 s. Every hop between a non-ramp
// motorway/trunk edge and a ramp pays kCurvyHighwayRampPenalty (cost only,
// no time): a leave-and-rejoin loop pays it twice, a genuine motorway
// stretch once on and once off. Additive and >= 0, so admissible.
inline constexpr float kCurvyHighwayRampPenalty = 60.0f;

inline bool is_highway_class(baldr::RoadClass cls) {
  return cls == baldr::RoadClass::kMotorway || cls == baldr::RoadClass::kTrunk;
}

// True for a transition between a non-ramp motorway/trunk edge and a ramp,
// in either order. Symmetric, so the forward and the reverse search agree.
inline bool highway_ramp_transition(baldr::RoadClass pred_cls,
                                    bool pred_ramp,
                                    baldr::RoadClass cls,
                                    bool ramp) {
  return (ramp && !pred_ramp && is_highway_class(pred_cls)) ||
         (pred_ramp && !ramp && is_highway_class(cls));
}

// Patch 0025 (#193): city settings. "In a city" is decided per edge from
// the stock mjolnir density (0..15, road km per km^2 within 2 km, the
// average of the edge's two end nodes) -- the same threshold as Valhalla's
// own is_urban (density > kMaxRuralDensity 8). No tile change: every tile,
// server and on-device, already carries it.
inline constexpr uint32_t kCityDensity = 9;
// Discourage city driving: graded factor 1 + K * (density - 8), so density
// 9 pays 1+K and 11 pays 1+3K. Calibrated on the Salzburg pins (handoff
// routing-cost, K sweep 1/1.5/2/3).
inline constexpr float kCityAversion = 2.0f;
// Patch 0026 (#193 fix round, F3): the graded factor is CAPPED. Uncapped,
// density 11 paid 7x and 15 paid 15x, so a rider who lives in a city was
// sent on a 5x detour to reach the rural network (Salzburg-Liefering ->
// Bergheim, Sport Touring: 32.2 km instead of 6.4). Calibrated on the
// Gneis -> Bergheim Fastest bypass and that home trip (handoff
// routing-cost-fix, cap sweep at K = 2): every cap in [2, 4.5] keeps the
// bypass AND the home trip at its own 6.4 km; at 4.75 and above Sport
// Touring flips to the 32 km Freilassing detour (no in-between path), and
// below 2 Fastest from home goes back over density 10. 3.0 = density 9's
// own factor, the middle of that band.
inline constexpr float kCityAversionCap = 3.0f;

// Patch 0028: a motorway reads a lower density than the streets around it
// (fenced, few junctions, fields and noise walls within its 2 km), so a
// stretch that runs through a city's edge -- Salzburg's A1 at Liefering
// reads 7-8 -- never reached 9, and "allow motorways in cities" left it
// at the first exit. Motorway/trunk edges and ramps count as in a city from
// 7: the urban stretches of Salzburg, Vienna, Linz, Graz and Innsbruck read
// 7-12, open-country motorway 3-5 (A1 east of Eugendorf, A10 to Hallein).
inline constexpr uint32_t kCityMotorwayDensity = 7;

inline bool in_city(uint32_t density, baldr::RoadClass cls, bool ramp) {
  return density >= ((is_highway_class(cls) || ramp) ? kCityMotorwayDensity : kCityDensity);
}

inline bool in_city(const baldr::DirectedEdge* edge) {
  return in_city(edge->density(), edge->classification(), edge->use() == baldr::Use::kRamp);
}

// The discourage-city multiplier of one edge: 1.0 outside a city, on
// motorway/trunk class edges (urban motorways stay the way through, owner
// ruling B3) and on ramps (patch 0026: exit ramps are often classified
// `primary`, yet they belong to the motorway they serve); graded above and
// capped at `cap`. Always >= 1.0, so the heuristic stays admissible, and
// soft: a destination inside a city stays reachable.
inline float city_aversion_factor(uint32_t density,
                                  baldr::RoadClass cls,
                                  bool ramp = false,
                                  float k = kCityAversion,
                                  float cap = kCityAversionCap) {
  if (is_highway_class(cls) || ramp || !in_city(density, cls, ramp)) {
    return 1.0f;
  }
  const float f = 1.0f + k * static_cast<float>(density - (kCityDensity - 1));
  return std::max(1.0f, std::min(f, cap));
}

// Patch 0026 (#193 fix round, F1): "fastest in cities" must not make a
// city CHEAPER than the countryside to a curvy route. Out of a city the
// curvy costing charges base * sp * cm * tm * pm; a straight main road
// (primary, the class a city's through-routes are) pays
// sp(0) * primary = (1 + alpha * kCurvyDetourCap) * 1.76. An in-city edge
// under city_fastest costs its fastest-preset base times that same
// constant, so the city costs what a straight main road costs the curvy
// rider outside (Sport Touring 3.34x, Cruiser 2.55x, Twisty Hunter 4.27x).
// The cheaper straight-tertiary constant (sp(0) alone) still pulled Sport
// Touring and Cruiser through Salzburg on a pass-by trip (Hallein ->
// Mattsee, handoff routing-cost-fix). ONE constant per request, so the
// in-city route choice among city edges stays exactly the fastest
// preset's (a constant scale does not reorder paths). Always >= 1.0.
inline float city_fastest_scale(float curvy_alpha, const ClassMultipliers& w) {
  return straightness_penalty(0, curvy_alpha) * std::max(1.0f, w.primary);
}

// Preferred-trail multiplier (patch 0019). Edges in the preferred set (member)
// pay their base cost; edges outside it pay a flat `factor` per km. Admissible
// like every other scenic factor: the result is always >= 1.0, so it can only
// RAISE cost and never breaks the A* heuristic. `factor` <= 1.0 (strength Off)
// is a no-op even for non-members.
inline float preferred_edge_multiplier(bool member, float factor) {
  return (member || factor <= 1.0f) ? 1.0f : factor;
}

// Patch 0035 (Vamoto #209 phase 6): the rider's road-roughness tolerance as
// a cost. `max_roughness` is the worst baldr::Surface byte (0 paved_smooth ..
// 7 impassable) the rider is happy on; an edge `e` steps rougher pays
// 1 + kRoughnessStep * e^2, capped at kRoughnessCap. Within the tolerance the
// factor is exactly 1 (an adventure profile at 6 never notices it); one step
// above it doubles, two steps cost 5x. The cap keeps a forced rough first
// or last edge (a start on a gravel lane) from flooding the search: the
// avoid-unpaved trail factor already stacks on unpaved edges. >= 1, so
// admissible. kMaxRoughness (7) is the off value and the default.
inline constexpr uint32_t kMaxRoughness = 7;
inline constexpr float kRoughnessStep = 1.0f;
inline constexpr float kRoughnessCap = 6.0f;

inline bool rougher_than(baldr::Surface surface, uint32_t max_roughness) {
  return static_cast<uint32_t>(surface) > max_roughness;
}

inline float roughness_multiplier(baldr::Surface surface, uint32_t max_roughness) {
  if (!rougher_than(surface, max_roughness)) {
    return 1.0f;
  }
  const float e = static_cast<float>(static_cast<uint32_t>(surface) - max_roughness);
  return std::min(kRoughnessCap, 1.0f + kRoughnessStep * e * e);
}

// Patch 0036 (Vamoto #209 phase 7): minor-road hops. A loop or arc route
// left the B159 at Pfarrwerfen and the B164 at Muehlbach am Hochkoenig for a
// few hundred metres of side street and rejoined the same road. The loop
// layers (0032 nice-road weight up to 3x, jitter up to 1.7x) price a straight
// main road up per metre, while the junction kinks of a short village street
// read as curves; the turn costs that keep a plain route on the main road do
// not grow with the layers (the 0028 lesson), so two turns became cheaper
// than a few hundred metres of main road.
//
// Owner ruling 2026-10-03: a residential, living or service way never earns
// the curve reward, the nice-road discount or a favourable jitter draw, and
// costs a bit more on a loop or arc request; it must not come out cheaper
// than the main road beside it, yet a route starting, ending or crossing a
// town still uses it to reach good roads.
inline bool is_minor_road(baldr::RoadClass cls, baldr::Use use) {
  using baldr::Use;
  return cls == baldr::RoadClass::kResidential || cls == baldr::RoadClass::kServiceOther ||
         use == Use::kLivingStreet || use == Use::kServiceRoad || use == Use::kParkingAisle ||
         use == Use::kDriveway || use == Use::kAlley || use == Use::kDriveThru;
}

// The extra factor a minor road pays on a loop or arc request: with no
// curve, nice-road or jitter discount, residential (1.65) * 1.25 = 2.06
// stays above a straight primary (1.76) at the same speed.
inline constexpr float kMinorRoadLoopFactor = 1.25f;

// The hop guard: on a loop or arc request, every transition between a main
// road (motorway, trunk, primary, secondary; not a ramp) and a road below it
// (unclassified or lower, a minor road, or a ramp) pays kMainRoadHopCost,
// scaled like the edges by the request's nice-road weight (1 + w). A
// leave-and-rejoin pays it twice; a route that really turns off onto a small
// road pays it once (a few hundred metres of a boring road against
// kilometres of a good one; the harness's minor-road hops fell from 970 to
// 43 over 960 plans with loop fit and twisty share at phase-6 level, where
// 40 left 112). Cost only,
// no time; symmetric, so the forward and the reverse search agree; >= 0,
// so admissible. Tertiary roads are not below: leaving a main road for a
// curvy tertiary is the point of curvy routing.
inline constexpr float kMainRoadHopCost = 80.0f;

inline bool is_main_road(baldr::RoadClass cls, baldr::Use use) {
  using baldr::RoadClass;
  return (cls == RoadClass::kMotorway || cls == RoadClass::kTrunk ||
          cls == RoadClass::kPrimary || cls == RoadClass::kSecondary) &&
         use != baldr::Use::kRamp && use != baldr::Use::kTurnChannel && !is_minor_road(cls, use);
}

inline bool is_below_main_road(baldr::RoadClass cls, baldr::Use use) {
  using baldr::RoadClass;
  return use == baldr::Use::kRamp || is_minor_road(cls, use) ||
         static_cast<uint32_t>(cls) >= static_cast<uint32_t>(RoadClass::kUnclassified);
}

inline bool main_road_hop_transition(baldr::RoadClass pred_cls,
                                     baldr::Use pred_use,
                                     baldr::RoadClass cls,
                                     baldr::Use use) {
  return (is_main_road(pred_cls, pred_use) && is_below_main_road(cls, use)) ||
         (is_main_road(cls, use) && is_below_main_road(pred_cls, pred_use));
}

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_SCENIC_COST_HELPERS_H_
