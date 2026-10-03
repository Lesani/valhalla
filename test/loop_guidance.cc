// Unit tests for the loop-guidance cost layers (vamoto patches 0029+).
#include "sif/loop_guidance.h"

#include <gtest/gtest.h>

#include <vector>

using valhalla::midgard::PointLL;
using valhalla::sif::corridor_multiplier;
using valhalla::sif::CorridorGrid;

namespace {

// ---- corridor_multiplier (patch 0029) ----

TEST(CorridorMultiplier, InsideBandIsOne) {
  for (float d : {0.0f, 500.0f, 1000.0f}) {
    EXPECT_FLOAT_EQ(corridor_multiplier(d, 1000.0f, 1.0f, 3.0f), 1.0f) << "d=" << d;
  }
}

TEST(CorridorMultiplier, RisesBeyondBandUpToCap) {
  // One half width beyond the band adds `slope`.
  EXPECT_FLOAT_EQ(corridor_multiplier(2000.0f, 1000.0f, 1.0f, 3.0f), 2.0f);
  EXPECT_FLOAT_EQ(corridor_multiplier(2000.0f, 1000.0f, 0.5f, 3.0f), 1.5f);
  EXPECT_FLOAT_EQ(corridor_multiplier(50000.0f, 1000.0f, 1.0f, 3.0f), 3.0f);
}

TEST(CorridorMultiplier, NeverBelowOne) {
  for (float slope : {-1.0f, 0.0f, 0.5f, 2.0f}) {
    for (float cap : {0.2f, 1.0f, 3.0f}) {
      for (float d : {0.0f, 900.0f, 1500.0f, 1e6f}) {
        EXPECT_GE(corridor_multiplier(d, 1000.0f, slope, cap), 1.0f)
            << "slope=" << slope << " cap=" << cap << " d=" << d;
      }
    }
  }
}

TEST(CorridorMultiplier, NoWidthIsOff) {
  EXPECT_FLOAT_EQ(corridor_multiplier(1e6f, 0.0f, 1.0f, 3.0f), 1.0f);
}

// ---- CorridorGrid (patch 0029) ----

// A 20 km west-east line at 47.8N.
std::vector<PointLL> line() {
  return {PointLL(13.0, 47.8), PointLL(13.1335, 47.8), PointLL(13.267, 47.8)};
}

// A point `north_m` north of the line's middle.
PointLL north_of_middle(double north_m) {
  return PointLL(13.1335, 47.8 + north_m / 110574.0);
}

TEST(CorridorGrid, OneInsideTheBand) {
  CorridorGrid g(line(), 2000.0f, 1.0f, 3.0f);
  ASSERT_FALSE(g.empty());
  EXPECT_FLOAT_EQ(g.factor(north_of_middle(0)), 1.0f);
  EXPECT_FLOAT_EQ(g.factor(north_of_middle(1500)), 1.0f);
}

TEST(CorridorGrid, MatchesTheMultiplierWithinOneCell) {
  CorridorGrid g(line(), 2000.0f, 1.0f, 3.0f);
  for (double d : {3000.0, 4000.0, 5000.0}) {
    const float want = corridor_multiplier(static_cast<float>(d), 2000.0f, 1.0f, 3.0f);
    // A cell is at most half_width / 4 = 500 m, so the error is under
    // 500 m * slope / half_width = 0.25.
    EXPECT_NEAR(g.factor(north_of_middle(d)), want, 0.25f) << "d=" << d;
  }
}

TEST(CorridorGrid, CapFarAwayAndOutsideTheGrid) {
  CorridorGrid g(line(), 2000.0f, 1.0f, 3.0f);
  EXPECT_FLOAT_EQ(g.factor(north_of_middle(9000)), 3.0f);
  EXPECT_FLOAT_EQ(g.factor(PointLL(16.0, 50.0)), 3.0f);
  EXPECT_FLOAT_EQ(g.factor(PointLL(10.0, 45.0)), 3.0f);
}

TEST(CorridorGrid, NonDecreasingAwayFromTheLine) {
  CorridorGrid g(line(), 1000.0f, 0.7f, 4.0f);
  float last = 1.0f;
  for (double d = 0; d <= 10000; d += 250) {
    const float f = g.factor(north_of_middle(d));
    EXPECT_GE(f + 1e-6f, last) << "d=" << d;
    EXPECT_GE(f, 1.0f);
    last = f;
  }
}

TEST(CorridorGrid, EmptyLineOrNoWidthIsOff) {
  EXPECT_TRUE(CorridorGrid({}, 1000.0f, 1.0f, 3.0f).empty());
  CorridorGrid off(line(), 0.0f, 1.0f, 3.0f);
  EXPECT_TRUE(off.empty());
  EXPECT_FLOAT_EQ(off.factor(PointLL(16.0, 50.0)), 1.0f);
}

TEST(CorridorGrid, HugeShapeStaysBounded) {
  // A 600 x 550 km box with a narrow band would need far more than
  // kMaxCells at the default cell size; the grid grows its cells instead.
  CorridorGrid g({PointLL(5.0, 42.0), PointLL(13.0, 47.0)}, 300.0f, 1.0f, 3.0f);
  EXPECT_GT(g.cell_m(), 300.0f / 4.0f);
  EXPECT_FLOAT_EQ(g.factor(PointLL(5.0, 42.0)), 1.0f);
}

} // namespace
