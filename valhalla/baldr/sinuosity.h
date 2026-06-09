#ifndef VALHALLA_BALDR_SINUOSITY_H_
#define VALHALLA_BALDR_SINUOSITY_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

// ---------------------------------------------------------------------------
// PoC-validated curviness metric (better_mc_routing v1.1, Issue #19).
//
// The v1 metric was whole-edge arc/chord, which under-scores edges that mix
// straight approaches with serpentine sections (the chord of the straight
// part dominates) and over-scores single gentle bends. The replacement is a
// blend of two components validated against hand-labelled rides in the PoC:
//
//   byte = round(255 * (0.59 * s_win + 0.41 * t))
//
//   s_win — mean windowed sinuosity: arc/chord over 500 m windows with 50%
//           overlap (one window every 250 m), normalized via
//           s_win = clamp((mean_sin - 1) / 2, 0, 1).
//   t     — turn density: interior shape points whose absolute bearing
//           change exceeds 25 deg, per km, normalized via
//           t = clamp((turns_per_km) / 25, 0, 1).
//
// The Python parity module (scripts/, main repo) implements the SAME
// constants verbatim — keep them in sync.
// ---------------------------------------------------------------------------

// Edges shorter than this carry no meaningful curviness signal -> byte 0.
inline constexpr double kSinuosityMinEdgeMeters = 100.0;
// Window geometry: 500 m windows, one starting every 250 m (50% overlap).
inline constexpr double kSinuosityWindowMeters = 500.0;
inline constexpr double kSinuosityWindowStrideMeters = 250.0;
// Windows whose chord collapses below this are treated as sinuosity 1.0.
inline constexpr double kSinuosityMinChordMeters = 1.0;
// Normalization of the mean window sinuosity: (mean - 1) / 2 -> [0, 1].
inline constexpr double kSinuosityWindowNorm = 2.0;
// Turn density: bearing changes above this count as turns ...
inline constexpr double kSinuosityTurnThresholdDeg = 25.0;
// ... and this many turns per km saturate the turn component at 1.0.
inline constexpr double kSinuosityTurnsPerKmNorm = 25.0;
// Blend weights (PoC grid search).
inline constexpr double kSinuosityWindowWeight = 0.59;
inline constexpr double kSinuosityTurnWeight = 0.41;

// (length_meters, sinuosity_byte) input pair for the shortcut aggregator.
// Decoupled from DirectedEdge so the aggregator is a pure compute function
// (PRD's `span<const DirectedEdge*>` would force a tile-lookup dep).
struct EdgeSinuosity {
  uint32_t length_m;
  uint8_t byte;
};

/**
 * Aggregate the sinuosity of multiple base edges into a single shortcut byte
 * using a length-weighted mean.
 *
 * Because the byte<->raw quantization is linear and uniform, weighting the
 * bytes directly is mathematically equivalent to weighting the inverse-decoded
 * raw values and re-quantizing — but without floating-point error or two
 * extra divides per edge.
 *
 * Empty input (defensive) returns 0.
 */
inline uint8_t aggregate_shortcut_sinuosity(std::span<const EdgeSinuosity> base_edges) {
  if (base_edges.empty()) {
    return 0;
  }
  uint64_t weighted_sum = 0;
  uint64_t total_length = 0;
  for (const auto& e : base_edges) {
    weighted_sum += static_cast<uint64_t>(e.length_m) * e.byte;
    total_length += e.length_m;
  }
  if (total_length == 0) {
    return 0;
  }
  return static_cast<uint8_t>(weighted_sum / total_length);
}

/**
 * Compute the quantized sinuosity byte for a shape polyline using the
 * PoC-validated windowed-sinuosity + turn-density blend (see file header).
 *
 * Defensive defaults that map to "no signal" (byte 0):
 *   - fewer than 2 points
 *   - total arc length below kSinuosityMinEdgeMeters (100 m)
 *
 * Distances are great-circle (PointLL::Distance — haversine on a sphere of
 * radius midgard::kRadEarthMeters); bearings are great-circle initial
 * bearings (PointLL::Heading). Window endpoints are linear lat/lng
 * interpolations along the polyline at the target cumulative arc distance.
 *
 * Pure function — no globals, no tile reads — directly unit-testable.
 */
inline uint8_t compute_sinuosity_byte(std::span<const midgard::PointLL> shape) {
  if (shape.size() < 2) {
    return 0;
  }

  // Cumulative arc length at every shape point.
  std::vector<double> cum(shape.size(), 0.0);
  for (size_t i = 1; i < shape.size(); ++i) {
    cum[i] = cum[i - 1] + shape[i - 1].Distance(shape[i]);
  }
  const double total = cum.back();
  if (total < kSinuosityMinEdgeMeters) {
    return 0;
  }

  // Linearly interpolated position along the polyline at arc distance d.
  auto point_at = [&shape, &cum](double d) -> midgard::PointLL {
    if (d <= 0.0) {
      return shape.front();
    }
    if (d >= cum.back()) {
      return shape.back();
    }
    // First point strictly past d; the segment [i, i+1] contains d.
    const auto it = std::upper_bound(cum.begin(), cum.end(), d);
    const size_t i = std::min(static_cast<size_t>(it - cum.begin() - 1), shape.size() - 2);
    const double seg = cum[i + 1] - cum[i];
    const double f = seg <= 0.0 ? 0.0 : (d - cum[i]) / seg;
    return {shape[i].lng() + f * (shape[i + 1].lng() - shape[i].lng()),
            shape[i].lat() + f * (shape[i + 1].lat() - shape[i].lat())};
  };

  // Windowed sinuosity: 500 m windows starting every 250 m of arc length.
  // A trailing window shorter than the 250 m stride is skipped when an
  // earlier window exists — its arc is mostly covered by the previous,
  // overlapping window. (The first window is always kept so edges in
  // [100 m, 250 m) still produce a measurement.)
  double sin_sum = 0.0;
  size_t sin_count = 0;
  for (double s = 0.0; s < total; s += kSinuosityWindowStrideMeters) {
    const double end = std::min(s + kSinuosityWindowMeters, total);
    const double window_arc = end - s;
    if (s > 0.0 && window_arc < kSinuosityWindowStrideMeters) {
      break;
    }
    const midgard::PointLL p0 = point_at(s);
    const midgard::PointLL p1 = point_at(end);
    const double chord = p0.Distance(p1);
    sin_sum += chord < kSinuosityMinChordMeters ? 1.0 : window_arc / chord;
    ++sin_count;
  }
  const double mean_sin = sin_sum / static_cast<double>(sin_count);
  const double s_win = std::clamp((mean_sin - 1.0) / kSinuosityWindowNorm, 0.0, 1.0);

  // Turn density: count interior shape points whose absolute bearing change
  // (wrapped to [0, 180]) exceeds the threshold. Degenerate (zero-length)
  // segments have no bearing; skip those points.
  uint32_t turns = 0;
  for (size_t i = 1; i + 1 < shape.size(); ++i) {
    if (shape[i] == shape[i - 1] || shape[i + 1] == shape[i]) {
      continue;
    }
    const double bearing_in = shape[i - 1].Heading(shape[i]);
    const double bearing_out = shape[i].Heading(shape[i + 1]);
    double delta = std::fabs(bearing_out - bearing_in);
    delta = std::fmod(delta, 360.0);
    if (delta > 180.0) {
      delta = 360.0 - delta;
    }
    if (delta > kSinuosityTurnThresholdDeg) {
      ++turns;
    }
  }
  const double turns_per_km = static_cast<double>(turns) / (total / 1000.0);
  const double t = std::clamp(turns_per_km / kSinuosityTurnsPerKmNorm, 0.0, 1.0);

  const double blended = kSinuosityWindowWeight * s_win + kSinuosityTurnWeight * t;
  return static_cast<uint8_t>(
      std::clamp(static_cast<int>(std::lround(255.0 * blended)), 0, 255));
}

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SINUOSITY_H_
