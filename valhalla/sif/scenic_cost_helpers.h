#ifndef VALHALLA_SIF_SCENIC_COST_HELPERS_H_
#define VALHALLA_SIF_SCENIC_COST_HELPERS_H_

#include <algorithm>
#include <cstdint>

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

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_SCENIC_COST_HELPERS_H_
