#ifndef VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_
#define VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_

// Pure helpers for the Layer-1 scenic-stretch extractor (better_mc_routing v2,
// Issue 02 — Slice 1). Decoupled from GraphReader / GraphTile so each helper
// can be unit-tested with synthetic inputs (vector of EdgeCandidate); the
// binary at src/mjolnir/valhalla_extract_stretches.cc is the I/O wrapper.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/sif/scenic_cost_helpers.h>

namespace valhalla {
namespace mjolnir {

// ---------------------------------------------------------------------------
// Curve metric (Menger curvature / radius-binning) — the v3 "what is a scenic
// road" signal, replacing per-edge arc/chord sinuosity. Adapted from Adam
// Franco's open-source roadcurvature project (roadcurvature.com): the local
// curve radius at each interior shape point is the circumradius of the triangle
// formed by it and its two neighbours; each segment is weighted by a
// radius-bin, and the weighted length summed gives "metres spent in turns".
// Per-edge sinuosity was blind to real roads (hairpins separated by straights
// average to ~0); curve density + a straight-tolerance fixes that.
// ---------------------------------------------------------------------------

// Curve-radius bins (metres) -> weight; tighter curve counts more.
inline float curve_weight_for_radius(double radius_m) {
  if (radius_m < 30.0) return 2.0f;
  if (radius_m < 60.0) return 1.6f;
  if (radius_m < 100.0) return 1.3f;
  if (radius_m < 175.0) return 1.0f;
  return 0.0f; // >= 175 m counts as straight
}

// Menger circumradius from three geodesic side lengths (a,b,c). Returns +inf
// for a degenerate (collinear) triple — i.e. a straight, infinite-radius curve.
inline double curve_circumradius(double a, double b, double c) {
  const double d =
      std::sqrt(std::fabs((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c)));
  return d == 0.0 ? std::numeric_limits<double>::infinity() : (a * b * c) / d;
}

// Radius-bin INDEX for the D4 histogram. Shares the 30/60/100/175 m
// thresholds with curve_weight_for_radius above -- one set of constants, two
// readings of it (a weight for the scalar density, an index for the shape of
// the distribution). Index 4 is the ">= 175 m" straight sink: it is
// accumulated so bin lengths add up to the shape length, but it is NOT part
// of the four emitted shares.
inline size_t curve_bin_for_radius(double radius_m) {
  if (radius_m < 30.0) return 0;
  if (radius_m < 60.0) return 1;
  if (radius_m < 100.0) return 2;
  if (radius_m < 175.0) return 3;
  return 4; // straight
}

// Curve metric of a polyline shape.
struct ShapeCurve {
  float curvy_m{0.0f};        // sum of segment_length * radius-bin weight
  float length_m{0.0f};       // total geodesic length
  float max_straight_m{0.0f}; // longest continuous weight-0 (straight) run
};

// Compute the curve metric of a shape. For each interior point the circumradius
// of (prev, this, next) is the local curve radius; it applies to the two
// segments around the point and the SMALLER of two competing radii wins.
inline ShapeCurve compute_shape_curve(const std::vector<midgard::PointLL>& shape) {
  ShapeCurve out;
  const size_t n = shape.size();
  if (n < 2) {
    return out;
  }
  std::vector<double> seglen(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    seglen[i] = shape[i].Distance(shape[i + 1]);
  }
  std::vector<double> segrad(n - 1, std::numeric_limits<double>::infinity());
  for (size_t i = 1; i + 1 < n; ++i) {
    const double base = shape[i - 1].Distance(shape[i + 1]);
    const double r = curve_circumradius(seglen[i - 1], seglen[i], base);
    segrad[i - 1] = std::min(segrad[i - 1], r);
    segrad[i] = std::min(segrad[i], r);
  }
  double run = 0.0;
  for (size_t i = 0; i + 1 < n; ++i) {
    out.length_m += static_cast<float>(seglen[i]);
    const float w = curve_weight_for_radius(segrad[i]);
    out.curvy_m += static_cast<float>(seglen[i] * w);
    if (w == 0.0f) {
      run += seglen[i];
      out.max_straight_m = std::max(out.max_straight_m, static_cast<float>(run));
    } else {
      run = 0.0;
    }
  }
  return out;
}

// Curve density = weighted curve metres / length. ~0 = straight, ~0.5 = a great
// pass (Stelvio measured 0.64), can exceed 1.0 where the tightest hairpins
// (weight 2.0) dominate. The scenic-road attractiveness base signal.
inline float curve_density(const ShapeCurve& c) {
  return c.length_m > 0.0f ? c.curvy_m / c.length_m : 0.0f;
}

// ---------------------------------------------------------------------------
// D4 radius-bucket histogram (WS-C1). The scalar density above says HOW curvy
// a road is; the histogram says WHAT KIND of curvy -- the share of the
// curve-classified length spent in each radius bin. Profiles multiply the
// shares by per-bin weights, so a hairpin road and a sweeper road with the
// same density can rank differently.
//
// IMPORTANT: the histogram is a SIDE CHANNEL. compute_shape_curve, curvy_m,
// curve_density, the seed/grow/emit gates and `score` are deliberately
// UNTOUCHED by this addition -- the emitted index composition must stay
// identical to the pre-bucket extractor, and the Europe-wide emitted stretch
// count (96,050) is the canary for that. If a re-extract moves that number,
// something in this file leaked into the scoring path.
// ---------------------------------------------------------------------------

// stretches.bin wire-format version. Bumped to 2 by the D4/D5 fields.
// Loaders MUST reject anything else (foundation mode: no dual-read path).
inline constexpr uint32_t kStretchFormatVersion = 2;

// Arc-length resample step for the histogram, in metres. MANDATORY per the
// WS-C1 gate (validation/reports/ws_c1_gate.md section 1): Valhalla tile
// shapes carry ~1e-6-degree (~0.11 m) coordinate quantization, and for three
// near-collinear points spaced d apart with lateral wobble h the Menger
// circumradius is about d^2/(2h). At the p10 native spacing of ~6 m that is
// R ~ 163 m, so quantization noise ALONE would file dead-straight road into
// the 100-175 m bin. At 20 m the same wobble gives R ~ 1800 m -- safely
// straight. 20 m also sits just above the native median spacing (16.8 m), so
// little real detail is lost, and it is fixed rather than data-dependent so
// shares stay comparable across stretches and regions (D8).
inline constexpr double kBucketResampleStepM = 20.0;

// Number of EMITTED bins (the straight sink is not emitted).
inline constexpr size_t kBucketCount = 4;

// Walk a polyline emitting a point every `step_m` of arc length. Endpoints are
// always kept, so the resampled shape spans the same road. Interpolation is
// linear in lat/lon, which is exact enough at 20 m: the great-circle sagitta
// over a 20 m chord is well under a micrometre. Shapes with fewer than two
// points are returned unchanged. Mirrors validation/prototype_buckets.py's
// resample_by_arclength (the gate implementation) step for step, carry
// included, so the offline harness and the extractor agree.
inline std::vector<midgard::PointLL>
resample_by_arclength(const std::vector<midgard::PointLL>& shape,
                      double step_m = kBucketResampleStepM) {
  if (shape.size() < 2 || !(step_m > 0.0)) {
    return shape;
  }
  std::vector<midgard::PointLL> out;
  out.reserve(shape.size());
  out.push_back(shape.front());
  double carry = 0.0; // arc length already walked since the last emitted point
  for (size_t i = 0; i + 1 < shape.size(); ++i) {
    const midgard::PointLL& a = shape[i];
    const midgard::PointLL& b = shape[i + 1];
    const double seg = a.Distance(b);
    if (seg <= 0.0) continue;
    double pos = step_m - carry;
    while (pos <= seg) {
      const double t = pos / seg;
      out.emplace_back(a.lng() + (b.lng() - a.lng()) * t,
                       a.lat() + (b.lat() - a.lat()) * t);
      pos += step_m;
    }
    carry = seg - (pos - step_m);
  }
  const midgard::PointLL& last = shape.back();
  if (out.back().lat() != last.lat() || out.back().lng() != last.lng()) {
    out.push_back(last);
  }
  return out;
}

// Per-radius-bin length of a shape, in metres. Five entries: the four curve
// bins then the ">= 175 m" straight accumulator, so the entries sum to the
// resampled shape length.
//
// Same walk as compute_shape_curve -- for each interior point the circumradius
// of (prev, this, next) is the local radius, it applies to the two segments
// either side of the point, and the SMALLER of two competing radii wins for a
// segment -- but computed on the 20 m resample and keeping the per-bin length
// instead of collapsing it into curvy_m.
inline std::array<float, kBucketCount + 1>
shape_bucket_lengths(const std::vector<midgard::PointLL>& shape,
                     double step_m = kBucketResampleStepM) {
  std::array<float, kBucketCount + 1> acc{};
  const std::vector<midgard::PointLL> pts = resample_by_arclength(shape, step_m);
  const size_t n = pts.size();
  if (n < 2) {
    return acc;
  }
  std::vector<double> seglen(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    seglen[i] = pts[i].Distance(pts[i + 1]);
  }
  std::vector<double> segrad(n - 1, std::numeric_limits<double>::infinity());
  for (size_t i = 1; i + 1 < n; ++i) {
    const double base = pts[i - 1].Distance(pts[i + 1]);
    const double r = curve_circumradius(seglen[i - 1], seglen[i], base);
    segrad[i - 1] = std::min(segrad[i - 1], r);
    segrad[i] = std::min(segrad[i], r);
  }
  for (size_t i = 0; i + 1 < n; ++i) {
    acc[curve_bin_for_radius(segrad[i])] += static_cast<float>(seglen[i]);
  }
  return acc;
}

// Curve-metric thresholds. Validated against the beloved-roads acceptance set:
// passes score 0.5-1.0, sweeper roads ~0.34, a pleasant rural Landstraße ~0.20,
// straight controls 0.09-0.12. A stretch must SEED on a clearly curvy edge and
// is EMITTED only if its overall density clears the floor; growth spans straights
// up to kMaxStraightRunMeters so hairpin-straight-hairpin chains stay together.
inline constexpr float kSeedCurveDensity = 0.18f;  // start on a clearly curvy edge
inline constexpr float kGrowCurveDensity = 0.05f;  // below this an edge is "straight"
// Emitted-stretch floor: the INDEX is curated highlights (passes + good twisty
// roads, density >= ~0.30); pleasant low-curve background Landstraßen are the
// COSTING's job, not the highlight index. Overridable via the -d CLI flag.
inline constexpr float kEmitCurveDensity = 0.30f;
inline constexpr float kMaxStraightRunMeters = 2400.0f; // ~1.5 mi straight ends a stretch

// Quantization-byte thresholds for the growth rules. The PRD specifies the
// SEED and GROW thresholds as wire bytes (102 and 76 respectively) — those
// are authoritative. Inverting the (raw - 1.0) * 127.5 quantization from
// valhalla/baldr/sinuosity.h gives the approximate raw arc/chord ratios:
//   seed byte 102 -> raw ~1.8 (so a stretch starts on a clearly curvy edge)
//   grow byte  76 -> raw ~1.6 (looser when extending — still firmly curvy)
// The raw-ratio numbers in the PRD prose ("1.4 / 1.3") are approximate; the
// wire bytes are the contract. If a future PRD revision retunes the
// thresholds, update these constants — not the prose comment.
inline constexpr uint8_t kSeedSinuosityByte = 102;
inline constexpr uint8_t kGrowSinuosityByte = 76;

// Length filter window for emitted stretches. Stretches whose total length
// falls outside [1 km, 20 km] are dropped; long ones are split at the
// lowest-sinuosity interior break point and re-checked.
inline constexpr float kMinStretchKm = 3.0f;  // highlights are substantial roads
inline constexpr float kMaxStretchKm = 20.0f;

// Straight-blip tolerance: inside an otherwise-curvy stretch we tolerate
// AT MOST one below-`kGrowSinuosityByte` edge, AND no more than 200 m of
// such "straight" edges total.
inline constexpr uint32_t kMaxStraightBlipCount = 1;
inline constexpr uint32_t kMaxStraightBlipMeters = 200;

// One candidate base-edge for the extractor. Decoupled from baldr::DirectedEdge
// + EdgeInfo so unit tests can build vectors directly without a GraphTile.
struct EdgeCandidate {
  uint32_t length_m{0};            // edge length in meters
  uint8_t sinuosity_byte{0};       // EdgeInfo::sinuosity() (legacy; unused by v3 metric)
  float curvy_m{0.0f};             // weighted curve metres (compute_shape_curve)
  float max_straight_m{0.0f};      // longest continuous straight run within the edge
  baldr::RoadClass road_class{baldr::RoadClass::kInvalid};
  baldr::Use use{baldr::Use::kRoad};
  uint8_t surface{0};              // baldr::Surface byte (0=smooth .. 7=impassable)
  bool roundabout{false};          // skip — roundabouts break stretches
  bool restricted_access{false};   // skip — non-motorbike-allowed edges
  std::vector<midgard::PointLL> shape; // ordered start -> end of base edge
  // --- appended by patch 0022 (WS-C1). Data only: nothing below participates
  // in seeding, growth, splitting, curvy_m or score. ---
  // D4: per-radius-bin length of this edge's 20 m-resampled shape, metres.
  // Index 0..3 = the <30 / 30-60 / 60-100 / 100-175 m bins, index 4 = the
  // straight sink (>= 175 m).
  std::array<float, kBucketCount + 1> bucket_len_m{};
  // D5: the stock per-edge DirectedEdge::density() flag (0-15, relative road
  // density around the edge -- Valhalla's urban-ness proxy).
  uint8_t density{0};
  // D5: does this edge END at a junction? See make_candidate in
  // src/mjolnir/valhalla_extract_stretches.cc for the bound definition.
  bool junction_at_end{false};
};

// One emitted stretch: an ordered list of EdgeCandidate references plus
// the derived metrics. Polyline is the concatenated shape (de-duplicated at
// joins).
struct EmittedStretch {
  std::vector<midgard::PointLL> polyline;
  float length_km{0.0f};
  float mean_sinuosity_raw{0.0f}; // length-weighted mean of (1.0 + byte/127.5), in [1.0, 3.0]
  float score{0.0f};              // (mean_sinuosity_byte/255) * class_multiplier, normalized to [0..1]
  baldr::RoadClass road_class{baldr::RoadClass::kInvalid};
  uint8_t surface{0};             // worst (max) Surface byte across the stretch's edges
  // --- appended by patch 0022 (WS-C1) ---
  // D4: share of the CURVE-CLASSIFIED length in each radius bin, x255.
  // All-zero means "no classified curvature" and consumers read that as the
  // neutral 1.0 bucket-affinity factor.
  std::array<uint8_t, kBucketCount> bucket_shares{};
  float mean_density{0.0f};       // D5: length-weighted mean of density() (0-15)
  float junctions_per_km{0.0f};   // D5: junction-ending edges per km
};

// Length-weighted mean sinuosity byte across the given edges.
// Returns 0 if total length is 0 (defensive — should not happen in practice).
inline float length_weighted_mean_sinuosity_byte(std::span<const EdgeCandidate> edges) {
  uint64_t weighted_sum = 0;
  uint64_t total_length = 0;
  for (const auto& e : edges) {
    weighted_sum += static_cast<uint64_t>(e.length_m) * e.sinuosity_byte;
    total_length += e.length_m;
  }
  if (total_length == 0) {
    return 0.0f;
  }
  return static_cast<float>(weighted_sum) / static_cast<float>(total_length);
}

// D4: the four emitted bucket shares for a whole edge run, quantized to uint8.
// Per-edge bin lengths are summed over the chain span, then each of the four
// CURVE bins is expressed as a share of the curve-classified length (the
// straight sink is excluded, so the shares carry the SHAPE of the curviness
// distribution and leave "how curvy overall" to the density term). Shares sum
// to 255 up to rounding, which makes the profile normalization structural: a
// neutral profile (all weights 1.0) scores exactly 1.0.
//
// All-zero out means the run has no curve-classified length at all. That is
// the documented "no bucket data" sentinel -- consumers treat it as the
// neutral factor 1.0, never as "0% in every bin".
inline std::array<uint8_t, kBucketCount>
bucket_shares_q8(std::span<const EdgeCandidate> edges) {
  std::array<double, kBucketCount> sums{};
  double classified = 0.0;
  for (const auto& e : edges) {
    for (size_t b = 0; b < kBucketCount; ++b) {
      sums[b] += e.bucket_len_m[b];
      classified += e.bucket_len_m[b];
    }
  }
  std::array<uint8_t, kBucketCount> out{};
  if (classified <= 0.0) {
    return out;
  }
  for (size_t b = 0; b < kBucketCount; ++b) {
    const long q = std::lround(sums[b] / classified * 255.0);
    out[b] = static_cast<uint8_t>(std::clamp<long>(q, 0, 255));
  }
  return out;
}

// D5: length-weighted mean of the stock per-edge density() flag (0-15).
// Returns 0 if the run has no length (defensive).
inline float length_weighted_mean_density(std::span<const EdgeCandidate> edges) {
  uint64_t weighted_sum = 0;
  uint64_t total_length = 0;
  for (const auto& e : edges) {
    weighted_sum += static_cast<uint64_t>(e.length_m) * e.density;
    total_length += e.length_m;
  }
  if (total_length == 0) {
    return 0.0f;
  }
  return static_cast<float>(weighted_sum) / static_cast<float>(total_length);
}

// D5: junction-ending edges per kilometre across the run. The run's LAST edge
// is counted like any other: its end node is where the stretch hands the rider
// over to the rest of the network, which is exactly the interruption D5 wants
// to measure. Returns 0 if the run has no length.
inline float junctions_per_km(std::span<const EdgeCandidate> edges) {
  uint32_t junctions = 0;
  uint64_t total_length = 0;
  for (const auto& e : edges) {
    if (e.junction_at_end) junctions++;
    total_length += e.length_m;
  }
  if (total_length == 0) {
    return 0.0f;
  }
  return static_cast<float>(junctions) * 1000.0f / static_cast<float>(total_length);
}

// PRD-defined score:
//   score_raw = (mean_byte / 255) * class_multiplier(road_class)
// `class_multiplier` returns up to ~5.0 (motorway) or down to ~0.85 (tertiary).
// To land in [0..1], normalize by the maximum class multiplier so the score
// is monotonic in sinuosity AND distinguishes road classes without inflating
// above 1.0. We use the same `ClassMultipliers` defaults as v1's curvy
// costing so the index agrees with the runtime preferences.
//
// NOTE: We use the lookup unweighted-by-Use here (pass kRoad) for the
// score; the granular Use overrides (kTrack, kLivingStreet) act as filters
// upstream — stretches don't form across those Uses anyway.
inline float compute_score(float mean_sinuosity_byte,
                           baldr::RoadClass cls,
                           const sif::ClassMultipliers& weights = sif::ClassMultipliers{}) {
  // Maximum value of class_multiplier across all RoadClass values for kRoad use.
  // Hardcoded from the ClassMultipliers struct (motorway = 5.0). We treat this
  // as the normalization constant; if the defaults are tuned later, this stays
  // correct as long as motorway remains the global max.
  const float max_cls = std::max({weights.motorway, weights.trunk, weights.primary,
                                  weights.secondary, weights.tertiary, weights.unclassified,
                                  weights.residential, weights.living_street, weights.service,
                                  weights.track});
  const float sinuosity_norm = mean_sinuosity_byte / 255.0f;
  const float class_term = sif::class_multiplier(cls, baldr::Use::kRoad, weights);
  return std::clamp(sinuosity_norm * (class_term / max_cls), 0.0f, 1.0f);
}

// Decode the length-weighted mean BYTE back to a raw arc/chord ratio for the
// `mean_sinuosity` proto field. Linear inverse of the quantization in
// valhalla/baldr/sinuosity.h (raw = 1.0 + byte / 127.5).
inline float byte_to_raw_sinuosity(float mean_byte) {
  return 1.0f + (mean_byte / 127.5f);
}

// Helper: total length in meters of an edge run.
inline uint32_t total_length_m(std::span<const EdgeCandidate> edges) {
  uint32_t sum = 0;
  for (const auto& e : edges) {
    sum += e.length_m;
  }
  return sum;
}

// Curve density of a whole stretch = sum(curvy_m) / sum(length_m). This is the
// v3 scenic-attractiveness base signal (replaces mean sinuosity).
inline float stretch_curve_density(std::span<const EdgeCandidate> edges) {
  float curvy = 0.0f;
  uint32_t len = 0;
  for (const auto& e : edges) {
    curvy += e.curvy_m;
    len += e.length_m;
  }
  return len > 0 ? curvy / static_cast<float>(len) : 0.0f;
}

// Helper: count below-grow-threshold "straight blip" edges and their total length.
struct BlipStats {
  uint32_t count{0};
  uint32_t total_meters{0};
};
inline BlipStats blip_stats(std::span<const EdgeCandidate> edges) {
  BlipStats s;
  for (const auto& e : edges) {
    if (e.sinuosity_byte < kGrowSinuosityByte) {
      s.count++;
      s.total_meters += e.length_m;
    }
  }
  return s;
}

// Helper: does an edge satisfy the "extend an existing stretch" predicate?
// (Loose check — used DURING growth. The straight-blip BUDGET is tracked by
// the caller; this just gates per-edge "yes it's eligible" vs "no, hard
// break".)
inline bool extension_eligible(const EdgeCandidate& e,
                               baldr::RoadClass anchor_class) {
  if (e.road_class != anchor_class) return false;
  if (e.roundabout) return false;
  if (e.restricted_access) return false;
  // Below-grow-threshold edges are eligible IF the budget allows — caller
  // checks the budget. Returning true here makes the caller responsible.
  return true;
}

// Split a stretch at its lowest-sinuosity interior edge. Returns the two
// halves. The break edge is dropped (it's the worst-curvy point so it
// represents the natural seam). If the stretch is too short to split
// (<3 edges), returns {original, empty}.
inline std::pair<std::vector<EdgeCandidate>, std::vector<EdgeCandidate>>
split_at_lowest_sinuosity(std::span<const EdgeCandidate> edges) {
  if (edges.size() < 3) {
    return {std::vector<EdgeCandidate>(edges.begin(), edges.end()), {}};
  }
  // Search only INTERIOR edges (index 1..n-2) so neither half is empty.
  size_t worst_idx = 1;
  uint8_t worst_byte = edges[1].sinuosity_byte;
  for (size_t i = 2; i + 1 < edges.size(); ++i) {
    if (edges[i].sinuosity_byte < worst_byte) {
      worst_byte = edges[i].sinuosity_byte;
      worst_idx = i;
    }
  }
  std::vector<EdgeCandidate> a(edges.begin(), edges.begin() + worst_idx);
  std::vector<EdgeCandidate> b(edges.begin() + worst_idx + 1, edges.end());
  return {std::move(a), std::move(b)};
}

// Concatenate edge shapes into one polyline, deduplicating join points
// (the last point of edge i typically equals the first point of edge i+1).
inline std::vector<midgard::PointLL>
concatenate_shapes(std::span<const EdgeCandidate> edges) {
  std::vector<midgard::PointLL> out;
  for (size_t i = 0; i < edges.size(); ++i) {
    const auto& s = edges[i].shape;
    if (s.empty()) continue;
    size_t start = 0;
    if (!out.empty() && !s.empty()) {
      const auto& last = out.back();
      const auto& first = s.front();
      // tolerate small float noise — within ~1 cm at the equator.
      if (std::abs(last.first - first.first) < 1e-7 &&
          std::abs(last.second - first.second) < 1e-7) {
        start = 1;
      }
    }
    out.insert(out.end(), s.begin() + start, s.end());
  }
  return out;
}

// Finalize a candidate edge run into one EmittedStretch (no length / blip
// check — those are upstream). Pure compute, no allocations beyond polyline.
inline EmittedStretch finalize(std::span<const EdgeCandidate> edges) {
  EmittedStretch out;
  if (edges.empty()) return out;
  out.length_km = total_length_m(edges) / 1000.0f;
  // v3: score = curve density (Menger radius-binning). road_class + surface are
  // carried on the stretch so the picker/costing apply class/surface weighting.
  // mean_sinuosity_raw is repurposed to carry the curve density into the QA dump.
  const float density = stretch_curve_density(edges);
  out.mean_sinuosity_raw = density;
  out.road_class = edges.front().road_class;
  out.score = density;
  out.polyline = concatenate_shapes(edges);
  uint8_t worst = 0;
  for (const auto& e : edges) {
    if (e.surface > worst) worst = e.surface;
  }
  out.surface = worst;
  // WS-C1 side channel (D4 + D5). Data only -- score above is already final.
  out.bucket_shares = bucket_shares_q8(edges);
  out.mean_density = length_weighted_mean_density(edges);
  out.junctions_per_km = junctions_per_km(edges);
  return out;
}

// Top-level emit decision: passes the length filter AND the blip tolerance.
// Long stretches are split recursively (caller responsibility) — this is the
// scalar check.
inline bool passes_emit_filters(std::span<const EdgeCandidate> edges) {
  if (edges.empty()) return false;
  const float km = total_length_m(edges) / 1000.0f;
  if (km < kMinStretchKm || km > kMaxStretchKm) return false;
  // v3: the stretch as a whole must clear the curve-density floor (a pleasant
  // rural Landstraße ~0.20; straight controls ~0.10).
  if (stretch_curve_density(edges) < kEmitCurveDensity) return false;
  return true;
}

// Take a raw candidate run that's already past the seed+grow phase and emit
// 0..N final stretches by:
//   1. If short — drop.
//   2. If in band — emit one.
//   3. If too long — split at lowest interior sinuosity, recurse on halves.
inline std::vector<EmittedStretch>
emit_with_splits(std::span<const EdgeCandidate> edges,
                 float min_density = kEmitCurveDensity) {
  std::vector<EmittedStretch> out;
  const float km = total_length_m(edges) / 1000.0f;
  if (km < kMinStretchKm) {
    return out;
  }
  // v3: the run must clear the curve-density floor to be a scenic stretch.
  // grow_forward already bounded straights (<= kMaxStraightRunMeters), so a run
  // that still falls below the floor is genuinely not curvy enough — drop it.
  if (stretch_curve_density(edges) < min_density) {
    return out;
  }
  if (km <= kMaxStretchKm) {
    out.push_back(finalize(edges));
    return out;
  }
  // Over the max length we can only get into band by splitting, and
  // split_at_lowest_sinuosity needs >= 3 edges to break the run (for shorter
  // runs it returns {original, empty}). A 1- or 2-edge run longer than the max
  // is therefore unsplittable and out of band — drop it. Without this guard
  // the recursion below re-enters on the *unchanged* span forever and blows
  // the stack: a single >20 km curvy edge does occur in the Europe tileset.
  if (edges.size() < 3) {
    return out;
  }
  // Long stretch: split at the lowest-sinuosity INTERIOR edge and recurse.
  auto [a, b] = split_at_lowest_sinuosity(edges);
  if (!a.empty()) {
    auto sub_a = emit_with_splits(std::span<const EdgeCandidate>(a), min_density);
    out.insert(out.end(), sub_a.begin(), sub_a.end());
  }
  if (!b.empty()) {
    auto sub_b = emit_with_splits(std::span<const EdgeCandidate>(b), min_density);
    out.insert(out.end(), sub_b.begin(), sub_b.end());
  }
  return out;
}

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_
