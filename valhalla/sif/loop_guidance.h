#ifndef VALHALLA_SIF_LOOP_GUIDANCE_H_
#define VALHALLA_SIF_LOOP_GUIDANCE_H_

// Loop guidance (vamoto patches 0029+): per-request cost layers that make a
// round trip FOLLOW a shape instead of driving to waypoints. Every multiplier
// here is >= 1.0, so the A* heuristic (calibrated on base costs) stays
// admissible, exactly like the curvy cost model (patch 0001) and the
// preferred-trail bias (patch 0019).

#include <valhalla/midgard/pointll.h>
#include <valhalla/sif/scenic_cost_helpers.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace valhalla {
namespace sif {

// Corridor multiplier (patch 0029): 1 inside the band (distance <= half
// width), then rising linearly by `slope` per half-width beyond it, capped at
// `cap`. Never below 1. A non-positive half width disables it.
inline float corridor_multiplier(float dist_m, float half_width_m, float slope, float cap) {
  if (half_width_m <= 0.0f || dist_m <= half_width_m) {
    return 1.0f;
  }
  const float f = 1.0f + std::max(0.0f, slope) * (dist_m - half_width_m) / half_width_m;
  return std::min(f, std::max(1.0f, cap));
}

// The corridor around one request's guide polyline, precomputed ONCE per
// request as a grid of multipliers, so the hot path is one array read per
// edge (no polyline scan per expansion). Local equirectangular metres around
// the polyline's bounding-box centre (fine at loop scale, < 400 km).
class CorridorGrid {
public:
  // Cells are about half_width / 4, clamped to [kMinCellM, kMaxCellM] and
  // grown further when the grid would exceed kMaxCells.
  static constexpr float kMinCellM = 60.0f;
  static constexpr float kMaxCellM = 600.0f;
  static constexpr size_t kMaxCells = 4u * 1000u * 1000u;

  CorridorGrid() = default;

  CorridorGrid(const std::vector<midgard::PointLL>& line,
               float half_width_m,
               float slope,
               float cap)
      : half_width_(half_width_m), slope_(std::max(0.0f, slope)), cap_(std::max(1.0f, cap)) {
    if (line.empty() || half_width_m <= 0.0f) {
      return;
    }
    // Beyond this distance the multiplier is the cap: no need to grid further.
    const float reach = slope_ > 0.0f ? half_width_ * (1.0f + (cap_ - 1.0f) / slope_) : half_width_;
    double min_lat = 90, max_lat = -90, min_lng = 180, max_lng = -180;
    for (const auto& p : line) {
      min_lat = std::min(min_lat, p.lat());
      max_lat = std::max(max_lat, p.lat());
      min_lng = std::min(min_lng, p.lng());
      max_lng = std::max(max_lng, p.lng());
    }
    lat0_ = 0.5 * (min_lat + max_lat);
    lng0_ = 0.5 * (min_lng + max_lng);
    mx_ = 111320.0 * std::cos(lat0_ * kPi / 180.0);
    my_ = 110574.0;
    // Grid origin and size in local metres, padded by the reach.
    x0_ = (min_lng - lng0_) * mx_ - reach;
    y0_ = (min_lat - lat0_) * my_ - reach;
    const double w = (max_lng - min_lng) * mx_ + 2.0 * reach;
    const double h = (max_lat - min_lat) * my_ + 2.0 * reach;
    cell_ = std::clamp(half_width_ / 4.0f, kMinCellM, kMaxCellM);
    while (static_cast<double>(std::ceil(w / cell_)) * std::ceil(h / cell_) >
           static_cast<double>(kMaxCells)) {
      cell_ *= 1.5f;
    }
    nx_ = static_cast<int32_t>(std::ceil(w / cell_)) + 1;
    ny_ = static_cast<int32_t>(std::ceil(h / cell_)) + 1;
    std::vector<float> dist(static_cast<size_t>(nx_) * ny_, std::numeric_limits<float>::max());
    // Every segment marks the cells within `reach` of it with the smaller
    // distance (a single point counts as a zero-length segment).
    std::vector<std::pair<double, double>> xy;
    xy.reserve(line.size());
    for (const auto& p : line) {
      xy.emplace_back((p.lng() - lng0_) * mx_, (p.lat() - lat0_) * my_);
    }
    const size_t nseg = xy.size() == 1 ? 1 : xy.size() - 1;
    for (size_t s = 0; s < nseg; ++s) {
      const auto& a = xy[s];
      const auto& b = xy.size() == 1 ? xy[s] : xy[s + 1];
      const double bx0 = std::min(a.first, b.first) - reach, bx1 = std::max(a.first, b.first) + reach;
      const double by0 = std::min(a.second, b.second) - reach,
                   by1 = std::max(a.second, b.second) + reach;
      const int32_t i0 = std::max(0, static_cast<int32_t>((bx0 - x0_) / cell_));
      const int32_t i1 = std::min(nx_ - 1, static_cast<int32_t>((bx1 - x0_) / cell_) + 1);
      const int32_t j0 = std::max(0, static_cast<int32_t>((by0 - y0_) / cell_));
      const int32_t j1 = std::min(ny_ - 1, static_cast<int32_t>((by1 - y0_) / cell_) + 1);
      const double dx = b.first - a.first, dy = b.second - a.second;
      const double len2 = dx * dx + dy * dy;
      for (int32_t j = j0; j <= j1; ++j) {
        const double py = y0_ + (j + 0.5) * cell_;
        for (int32_t i = i0; i <= i1; ++i) {
          const double px = x0_ + (i + 0.5) * cell_;
          double t = len2 > 0 ? ((px - a.first) * dx + (py - a.second) * dy) / len2 : 0.0;
          t = std::clamp(t, 0.0, 1.0);
          const double qx = a.first + t * dx - px, qy = a.second + t * dy - py;
          const float d = static_cast<float>(std::sqrt(qx * qx + qy * qy));
          float& cur = dist[static_cast<size_t>(j) * nx_ + i];
          cur = std::min(cur, d);
        }
      }
    }
    factor_.resize(dist.size());
    for (size_t k = 0; k < dist.size(); ++k) {
      factor_[k] = corridor_multiplier(dist[k], half_width_, slope_, cap_);
    }
  }

  bool empty() const {
    return factor_.empty();
  }

  // The multiplier at `ll`: the grid cell's, or the cap outside the grid.
  float factor(const midgard::PointLL& ll) const {
    if (factor_.empty()) {
      return 1.0f;
    }
    const double x = (ll.lng() - lng0_) * mx_ - x0_;
    const double y = (ll.lat() - lat0_) * my_ - y0_;
    if (x < 0 || y < 0) {
      return cap_;
    }
    const auto i = static_cast<int32_t>(x / cell_);
    const auto j = static_cast<int32_t>(y / cell_);
    if (i >= nx_ || j >= ny_) {
      return cap_;
    }
    return factor_[static_cast<size_t>(j) * nx_ + i];
  }

  float cell_m() const {
    return cell_;
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  float half_width_ = 0.0f;
  float slope_ = 0.0f;
  float cap_ = 1.0f;
  double lat0_ = 0, lng0_ = 0, mx_ = 1, my_ = 1, x0_ = 0, y0_ = 0;
  float cell_ = kMinCellM;
  int32_t nx_ = 0, ny_ = 0;
  std::vector<float> factor_;
};

// ---- seeded jitter (patch 0032) ----

// splitmix64 finaliser: a well-mixed 64-bit hash of `x`.
inline uint64_t mix64(uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

// A uniform value in [0, 1) for (key, seed): the same pair always gives the
// same value, any other seed an independent one.
inline float unit_hash(uint64_t key, uint32_t seed) {
  const uint64_t h = mix64(key ^ mix64(static_cast<uint64_t>(seed) + 0x5EEDull));
  return static_cast<float>(h >> 40) / static_cast<float>(1ull << 24);
}

// A smooth seeded field in [0, 1) over the plane (metres): value noise on a
// lattice of `cell_m`, smoothstep-interpolated, so neighbouring roads share
// most of their jitter and a whole valley can be cheaper for one seed.
inline float value_noise(double x_m, double y_m, float cell_m, uint32_t seed) {
  const double gx = x_m / cell_m, gy = y_m / cell_m;
  const double fx = std::floor(gx), fy = std::floor(gy);
  const auto ix = static_cast<int64_t>(fx), iy = static_cast<int64_t>(fy);
  const double tx = gx - fx, ty = gy - fy;
  const double sx = tx * tx * (3 - 2 * tx), sy = ty * ty * (3 - 2 * ty);
  auto at = [seed](int64_t i, int64_t j) {
    return static_cast<double>(
        unit_hash((static_cast<uint64_t>(i) << 32) ^ static_cast<uint64_t>(j & 0xFFFFFFFF), seed));
  };
  const double a = at(ix, iy) + sx * (at(ix + 1, iy) - at(ix, iy));
  const double b = at(ix, iy + 1) + sx * (at(ix + 1, iy + 1) - at(ix, iy + 1));
  return static_cast<float>(std::min(a + sy * (b - a), 0.9999999));
}

// The jitter multiplier for a field/hash value u in [0, 1): 1 + amount * u,
// never below 1 (keyed by the request seed only, never the profile).
inline float jitter_multiplier(float u, float amount) {
  return 1.0f + std::max(0.0f, amount) * std::clamp(u, 0.0f, 1.0f);
}

// ---- nice-road weight (patch 0032) ----

// Relative pricing from data the tiles carry: the per-edge curve-density byte
// (patch 0012) and the node density (urban). A curvy rural road is "nice" and
// pays x1; a straight or urban one pays up to 1 + weight. Boring is priced UP
// rather than nice DOWN, so the factor never drops below 1.
inline float nice_road_multiplier(uint8_t curve_byte, bool urban, float weight) {
  if (weight <= 0.0f) {
    return 1.0f;
  }
  const float n = urban ? 0.0f : std::min(1.0f, curve_byte / kCurveDensityFullByte);
  return 1.0f + weight * (1.0f - n);
}

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_LOOP_GUIDANCE_H_
