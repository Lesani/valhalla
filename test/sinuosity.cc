// Unit tests for the pure compute_sinuosity_byte helper.
// Issue 04 of better_mc_routing v1 introduced the whole-edge arc/chord
// metric; Issue #19 (v1.1) replaced it with the PoC-validated blend of
// windowed sinuosity (500 m windows, 50% overlap) and turn density.
//
// Expectations below were cross-checked against an independent Python
// replica of the metric. Where exact values depend on the details of the
// distance function (haversine on a sphere of radius kRadEarthMeters) the
// asserted ranges are generous (at least +/- 3 bytes).

#include "baldr/sinuosity.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using valhalla::baldr::compute_sinuosity_byte;
using valhalla::midgard::PointLL;

namespace {

// All test geometries live near (lng=0, lat=0) at small offsets so the
// lat/lon distortion in PointLL::Distance is negligible vs the math we
// expect. 0.001 deg of longitude at the equator is ~111.3 m.

// Build a tight serpentine: legs of ~35 m alternating between lat 0 and
// lat 0.0003 (~33 m swing) while stepping 0.0001 deg (~11 m) east per
// point. Both the windowed sinuosity and the turn density saturate.
std::vector<PointLL> tight_serpentine(int num_points, double start_lng = 0.0) {
  std::vector<PointLL> shape;
  shape.reserve(num_points);
  double lng = start_lng;
  for (int i = 0; i < num_points; ++i) {
    shape.emplace_back(lng, (i % 2) ? 0.0003 : 0.0);
    lng += 0.0001;
  }
  return shape;
}

TEST(Sinuosity, StraightTwoPointLine) {
  // Two points ~111 m apart along the equator: one (short) window with
  // arc == chord -> mean sinuosity 1.0, no turns -> byte 0.
  const std::vector<PointLL> shape{PointLL(0.0, 0.0), PointLL(0.001, 0.0)};
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, SinglePointDegenerate) {
  // Fewer than 2 points -> no signal (byte 0).
  const std::vector<PointLL> shape{PointLL(0.0, 0.0)};
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, SubHundredMeterEdgeIsZero) {
  // Total arc length below the 100 m guard -> byte 0 even though the
  // geometry doubles back on itself.
  const std::vector<PointLL> shape{
      PointLL(0.0, 0.0),
      PointLL(0.0003, 0.0),    // ~33 m east
      PointLL(0.0001, 0.0002), // back west and north, still < 100 m total
  };
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, ChordBelowOneMeter) {
  // Tiny loop: arc has some length but everything is sub-100 m -> the
  // length guard returns 0 before window chords are even evaluated.
  const std::vector<PointLL> shape{
      PointLL(0.0, 0.0),
      PointLL(0.00001, 0.0), // ~1.1 m to the east
      PointLL(1e-8, 0.0),    // back to ~1.1 mm from origin
  };
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, LongDeadStraightEdgeIsZero) {
  // 5 km of dead-straight road sampled every ~55 m: every window has
  // arc == chord and there are no turns -> byte 0.
  std::vector<PointLL> shape;
  for (int i = 0; i <= 90; ++i) {
    shape.emplace_back(0.0005 * i, 0.0);
  }
  EXPECT_EQ(compute_sinuosity_byte(shape), 0);
}

TEST(Sinuosity, SemicircleModerateScore) {
  // 16-segment polyline along a semicircle of ~111 m radius (arc ~350 m).
  // One 350 m window: sinuosity pi/2 ~ 1.5708 -> s_win ~ 0.285. The
  // 11.25 deg bearing steps stay below the 25 deg turn threshold -> t = 0.
  // byte = round(255 * 0.59 * 0.285) ~ 43.
  constexpr int N = 16;
  constexpr double R = 0.001; // ~111 m radius near the equator
  std::vector<PointLL> shape;
  shape.reserve(N + 1);
  for (int i = 0; i <= N; ++i) {
    const double angle = M_PI * i / N; // 0 .. pi
    shape.emplace_back(R * std::cos(angle), R * std::sin(angle));
  }
  const uint8_t byte = compute_sinuosity_byte(shape);
  EXPECT_GE(byte, 40);
  EXPECT_LE(byte, 46);
}

TEST(Sinuosity, TightSerpentineSaturates) {
  // ~1 km of 35 m switchbacks: every 500 m window has arc/chord > 3
  // (s_win = 1.0) and ~28 turns/km (t = 1.0) -> byte 255.
  const auto shape = tight_serpentine(30);
  const uint8_t byte = compute_sinuosity_byte(shape);
  EXPECT_GE(byte, 252);
}

TEST(Sinuosity, CoarseZigzagIsOnlyModeratelyCurvy) {
  // Kilometer-long zigzag legs (the old TightZigzagClipsTo255 geometry):
  // whole-edge arc/chord is >> 3 (old metric: byte 255), but at the 500 m
  // window scale the legs are straight and only the ~10 hairpins over
  // 11 km register -> the new metric scores it mid-range (~72), which is
  // the desired scale-awareness: long straights between hairpins are NOT
  // a serpentine road.
  std::vector<PointLL> shape{PointLL(0.0, 0.0)};
  for (int i = 0; i < 10; ++i) {
    shape.emplace_back(0.001 * i, (i % 2) ? 0.005 : -0.005);
  }
  shape.emplace_back(0.01, 0.0);
  const uint8_t byte = compute_sinuosity_byte(shape);
  EXPECT_GE(byte, 62);
  EXPECT_LE(byte, 82);
}

TEST(Sinuosity, CompositeStraightThenSerpentine) {
  // 3 km dead straight followed by ~1 km of tight serpentine. The
  // whole-edge arc/chord of this geometry is only ~1.21 (old-metric byte
  // ~26) because the straight chord dominates — the windowed metric must
  // score it clearly higher (replica value: 69) since a quarter of the
  // ride is full-on switchbacks.
  std::vector<PointLL> shape;
  for (int i = 0; i < 55; ++i) {
    shape.emplace_back(0.0005 * i, 0.0); // 3 km straight east
  }
  const auto serp = tight_serpentine(30, 0.0275);
  shape.insert(shape.end(), serp.begin(), serp.end());

  const uint8_t byte = compute_sinuosity_byte(shape);
  // Well above what the whole-edge arc/chord (byte ~26) would suggest.
  EXPECT_GE(byte, 55);
  EXPECT_LE(byte, 85);
}

// ---- aggregate_shortcut_sinuosity tests (unchanged by Issue #19) ----

using valhalla::baldr::aggregate_shortcut_sinuosity;
using valhalla::baldr::EdgeSinuosity;

TEST(Sinuosity, AggregateEqualLengthsMixedBytes) {
  // 3 equal-length base edges -> plain mean of the bytes (truncating).
  const std::vector<EdgeSinuosity> base{{1000, 0}, {1000, 127}, {1000, 63}};
  const uint8_t result = aggregate_shortcut_sinuosity(base);
  EXPECT_GE(result, 62);
  EXPECT_LE(result, 65);
}

TEST(Sinuosity, AggregateLengthSkewedTowardStraight) {
  // 1000 m at byte 0 + 10 m at byte 255:
  // weighted = (1000*0 + 10*255) / 1010 = 2.52 -> byte 2.
  const std::vector<EdgeSinuosity> base{{1000, 0}, {10, 255}};
  const uint8_t result = aggregate_shortcut_sinuosity(base);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4);
}

TEST(Sinuosity, AggregateAllEqualIsIdempotent) {
  // All base edges already at byte 100 -> aggregate stays at 100.
  const std::vector<EdgeSinuosity> base{{500, 100}, {1500, 100}, {200, 100}};
  EXPECT_EQ(aggregate_shortcut_sinuosity(base), 100);
}

TEST(Sinuosity, AggregateEmptyDefensiveZero) {
  const std::vector<EdgeSinuosity> base{};
  EXPECT_EQ(aggregate_shortcut_sinuosity(base), 0);
}

} // namespace
