#ifndef VALHALLA_SIF_SCENIC_COST_HELPERS_H_
#define VALHALLA_SIF_SCENIC_COST_HELPERS_H_

#include <algorithm>
#include <cstdint>

#include <valhalla/baldr/graphconstants.h>

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

/**
 * Per road-class cost multipliers for motorcycle_curvy (better_mc_routing v1).
 * Issue 07 — the Calimoto fix lives in the `residential` value: a high cost
 * weight there discourages routing through village residential streets even
 * when they happen to be slightly curvier than the main valley road.
 *
 * Values are compile-time defaults; not exposed in the v1 API surface.
 * `living_street`, `service`, `track` are Use enum sub-categories that
 * the helper checks BEFORE classification() — they override the RoadClass.
 */
struct ClassMultipliers {
  float motorway = 5.00f;
  float trunk = 3.00f;
  float primary = 1.50f;
  float secondary = 1.00f;
  float tertiary = 0.85f;
  float unclassified = 0.90f;
  float residential = 1.40f; // Calimoto fix
  float living_street = 2.00f;
  float service = 2.50f;
  float track = 5.00f;
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

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_SCENIC_COST_HELPERS_H_
