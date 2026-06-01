#ifndef VALHALLA_SIF_SCENIC_COST_HELPERS_H_
#define VALHALLA_SIF_SCENIC_COST_HELPERS_H_

#include <algorithm>
#include <cstdint>

namespace valhalla {
namespace sif {

/**
 * Multiplicative cost discount for curvy edges (better_mc_routing v1).
 *
 * Returns a factor in [1 - alpha, 1.0]:
 *   - sinuosity_byte = 0   (straight)  -> 1.0          (no discount)
 *   - sinuosity_byte = 255 (max curvy) -> 1 - alpha    (full discount)
 *   - linearly interpolated in between
 *
 * alpha = 0 disables the bonus regardless of sinuosity. alpha is expected
 * to come from the caller already clamped to [0.0, 0.95] (Issue 06's
 * ParseMotorcycleCurvyCostOptions does this); the function still produces
 * sane output for out-of-range alpha but the caller should not rely on it.
 *
 * Pure function — no globals, no allocations — directly unit-testable.
 */
inline float curvy_bonus(uint8_t sinuosity_byte, float alpha) {
  const float sinuosity_norm = sinuosity_byte / 255.0f;
  return 1.0f - alpha * sinuosity_norm;
}

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_SCENIC_COST_HELPERS_H_
