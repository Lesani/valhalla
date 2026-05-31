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
