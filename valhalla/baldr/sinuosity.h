#ifndef VALHALLA_BALDR_SINUOSITY_H_
#define VALHALLA_BALDR_SINUOSITY_H_

#include <algorithm>
#include <cstdint>
#include <span>

#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

// Quantization band: raw arc/chord in [1.0, 3.0] -> byte in [0, 255], linear.
// raw 1.0 -> byte 0,  raw 3.0 -> byte 255,  values above 3.0 clipped to 255.
inline constexpr float kSinuosityRawMin = 1.0f;
inline constexpr float kSinuosityRawMax = 3.0f;
inline constexpr float kSinuosityScale = 255.0f / (kSinuosityRawMax - kSinuosityRawMin); // 127.5

// Minimum chord length below which sinuosity is meaningless; treat as straight.
inline constexpr float kSinuosityMinChordMeters = 1.0f;

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
 * Compute the quantized sinuosity byte for a shape polyline.
 *
 * Math: arc / chord, where arc is the cumulative distance through shape points
 * and chord is the great-circle distance from first to last point.
 *
 * Defensive defaults that map to "straight" (byte 0):
 *   - fewer than 2 points
 *   - chord shorter than kSinuosityMinChordMeters
 *
 * Pure function — no globals, no tile reads — directly unit-testable.
 */
inline uint8_t compute_sinuosity_byte(std::span<const midgard::PointLL> shape) {
  if (shape.size() < 2) {
    return 0;
  }
  float arc = 0.0f;
  for (size_t i = 1; i < shape.size(); ++i) {
    arc += shape[i - 1].Distance(shape[i]);
  }
  const float chord = shape.front().Distance(shape.back());
  if (chord < kSinuosityMinChordMeters) {
    return 0;
  }
  const float raw = arc / chord;
  const int byte = static_cast<int>((raw - kSinuosityRawMin) * kSinuosityScale);
  return static_cast<uint8_t>(std::clamp(byte, 0, 255));
}

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SINUOSITY_H_
