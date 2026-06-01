// Unit tests for the pure compute_sinuosity_byte helper.
// Issue 04 of better_mc_routing v1 — the curvy-routing engine.

#include "baldr/sinuosity.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using valhalla::baldr::compute_sinuosity_byte;
using valhalla::midgard::PointLL;

namespace {

// All test geometries live near (lng=0, lat=0) at small offsets so the
// lat/lon distortion in PointLL::Distance is negligible vs the math we expect.

TEST(Sinuosity, StraightTwoPointLine) {
  // Two points 111m-ish apart along the equator -> arc == chord -> raw 1.0 -> byte 0.
  const std::vector<PointLL> shape{PointLL(0.0, 0.0), PointLL(0.001, 0.0)};
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, SinglePointDegenerate) {
  // Fewer than 2 points cannot have a chord; treat as straight (byte 0).
  const std::vector<PointLL> shape{PointLL(0.0, 0.0)};
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, ChordBelowOneMeter) {
  // Three points that start and end so close (<1 m apart) that chord
  // is sub-meter even though arc has some length. Treat as straight.
  // 1 deg of longitude at equator ~ 111 km, so 1e-8 deg ~ 1.1 mm.
  const std::vector<PointLL> shape{
      PointLL(0.0, 0.0),
      PointLL(0.00001, 0.0), // ~1.1 m to the east
      PointLL(1e-8, 0.0),    // back to ~1.1 mm from origin
  };
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, SemicircleApproximationByte72) {
  // 16-segment polyline along a semicircle of small radius near (0,0).
  // Expected raw arc/chord = pi/2 ~ 1.5708.
  // Byte = floor((1.5708 - 1) * 127.5) = 72 (or close to it after sphere math).
  constexpr int N = 16;
  constexpr double R = 0.001; // ~111 m radius near equator
  std::vector<PointLL> shape;
  shape.reserve(N + 1);
  for (int i = 0; i <= N; ++i) {
    const double angle = M_PI * i / N; // 0 .. pi
    shape.emplace_back(R * std::cos(angle), R * std::sin(angle));
  }
  const uint8_t byte = compute_sinuosity_byte(shape);
  // Tolerance covers great-circle vs Cartesian discrepancy at small scale.
  EXPECT_GE(byte, 65);
  EXPECT_LE(byte, 80);
}

// ---- aggregate_shortcut_sinuosity tests ----

using valhalla::baldr::aggregate_shortcut_sinuosity;
using valhalla::baldr::EdgeSinuosity;

TEST(Sinuosity, AggregateEqualLengthsMixedBytes) {
  // PRD case: 3 equal-length base edges with raw {1.0, 2.0, 1.5}.
  // Quantized bytes: 0, 127, 63 -> length-weighted mean = (0+127+63)/3 = 63.
  // PRD expected ~64; truncating quantization yields 63.
  const std::vector<EdgeSinuosity> base{{1000, 0}, {1000, 127}, {1000, 63}};
  const uint8_t result = aggregate_shortcut_sinuosity(base);
  EXPECT_GE(result, 62);
  EXPECT_LE(result, 65);
}

TEST(Sinuosity, AggregateLengthSkewedTowardStraight) {
  // PRD case: 1000m at raw 1.0 (byte 0) + 10m at raw 3.0 (byte 255).
  // weighted = (1000*0 + 10*255) / 1010 = 2.52 -> byte 2.
  const std::vector<EdgeSinuosity> base{{1000, 0}, {10, 255}};
  const uint8_t result = aggregate_shortcut_sinuosity(base);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4);
}

TEST(Sinuosity, AggregateAllEqualIsIdempotent) {
  // PRD case: all base edges already at byte 100 -> aggregate stays at 100.
  const std::vector<EdgeSinuosity> base{{500, 100}, {1500, 100}, {200, 100}};
  EXPECT_EQ(aggregate_shortcut_sinuosity(base), 100);
}

TEST(Sinuosity, AggregateEmptyDefensiveZero) {
  const std::vector<EdgeSinuosity> base{};
  EXPECT_EQ(aggregate_shortcut_sinuosity(base), 0);
}

TEST(Sinuosity, TightZigzagClipsTo255) {
  // Zigzag whose arc is much longer than its straight chord (raw >> 3.0).
  // Verifies the clip-above-3.0 branch returns 255.
  std::vector<PointLL> shape{PointLL(0.0, 0.0)};
  // Sawtooth: go far north, back south, far north, ... so arc piles up
  // while net horizontal chord stays small.
  for (int i = 0; i < 10; ++i) {
    shape.emplace_back(0.001 * i, (i % 2) ? 0.005 : -0.005);
  }
  shape.emplace_back(0.01, 0.0); // end near the chord baseline
  const uint8_t byte = compute_sinuosity_byte(shape);
  EXPECT_EQ(byte, 255);
}

} // namespace
