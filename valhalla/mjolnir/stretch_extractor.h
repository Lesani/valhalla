#ifndef VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_
#define VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_

// Pure helpers for the Layer-1 scenic-stretch extractor (better_mc_routing v2,
// Issue 02 — Slice 1). Decoupled from GraphReader / GraphTile so each helper
// can be unit-tested with synthetic inputs (vector of EdgeCandidate); the
// binary at src/mjolnir/valhalla_extract_stretches.cc is the I/O wrapper.

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/sif/scenic_cost_helpers.h>

namespace valhalla {
namespace mjolnir {

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
inline constexpr float kMinStretchKm = 1.0f;
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
  uint8_t sinuosity_byte{0};       // EdgeInfo::sinuosity()
  baldr::RoadClass road_class{baldr::RoadClass::kInvalid};
  baldr::Use use{baldr::Use::kRoad};
  uint8_t surface{0};              // baldr::Surface byte (0=smooth .. 7=impassable)
  bool roundabout{false};          // skip — roundabouts break stretches
  bool restricted_access{false};   // skip — non-motorbike-allowed edges
  std::vector<midgard::PointLL> shape; // ordered start -> end of base edge
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
  const float mean_byte = length_weighted_mean_sinuosity_byte(edges);
  out.length_km = total_length_m(edges) / 1000.0f;
  out.mean_sinuosity_raw = byte_to_raw_sinuosity(mean_byte);
  out.road_class = edges.front().road_class;
  out.score = compute_score(mean_byte, out.road_class);
  out.polyline = concatenate_shapes(edges);
  uint8_t worst = 0;
  for (const auto& e : edges) {
    if (e.surface > worst) worst = e.surface;
  }
  out.surface = worst;
  return out;
}

// Top-level emit decision: passes the length filter AND the blip tolerance.
// Long stretches are split recursively (caller responsibility) — this is the
// scalar check.
inline bool passes_emit_filters(std::span<const EdgeCandidate> edges) {
  if (edges.empty()) return false;
  const float km = total_length_m(edges) / 1000.0f;
  if (km < kMinStretchKm || km > kMaxStretchKm) return false;
  const BlipStats s = blip_stats(edges);
  if (s.count > kMaxStraightBlipCount) return false;
  if (s.total_meters > kMaxStraightBlipMeters) return false;
  return true;
}

// Take a raw candidate run that's already past the seed+grow phase and emit
// 0..N final stretches by:
//   1. If short — drop.
//   2. If in band — emit one.
//   3. If too long — split at lowest interior sinuosity, recurse on halves.
inline std::vector<EmittedStretch>
emit_with_splits(std::span<const EdgeCandidate> edges) {
  std::vector<EmittedStretch> out;
  const float km = total_length_m(edges) / 1000.0f;
  if (km < kMinStretchKm) {
    return out;
  }
  // Blip filter applies to the WHOLE run as-emitted; if violated, we drop
  // rather than split (the run is too noisy to be a clean stretch).
  const BlipStats s = blip_stats(edges);
  if (s.count > kMaxStraightBlipCount || s.total_meters > kMaxStraightBlipMeters) {
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
    auto sub_a = emit_with_splits(std::span<const EdgeCandidate>(a));
    out.insert(out.end(), sub_a.begin(), sub_a.end());
  }
  if (!b.empty()) {
    auto sub_b = emit_with_splits(std::span<const EdgeCandidate>(b));
    out.insert(out.end(), sub_b.begin(), sub_b.end());
  }
  return out;
}

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_STRETCH_EXTRACTOR_H_
