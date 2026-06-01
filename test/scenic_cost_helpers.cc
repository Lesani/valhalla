// Unit tests for the pure helpers in valhalla/sif/scenic_cost_helpers.h.
// better_mc_routing v1 — Issues 06 (curvy_bonus), 07 (class_multiplier),
// 08 (is_scenic_toll). One test file covers all three helpers; new TEST
// blocks are added as each issue lands its helper.

#include "sif/scenic_cost_helpers.h"

#include <gtest/gtest.h>

using valhalla::sif::curvy_bonus;

namespace {

// ---- curvy_bonus (Issue 06) ----

TEST(CurvyBonus, StraightEdgeNoDiscount) {
  // Straight edge -> no curvy bonus regardless of alpha.
  EXPECT_FLOAT_EQ(curvy_bonus(0, 0.6f), 1.0f);
  EXPECT_FLOAT_EQ(curvy_bonus(0, 0.0f), 1.0f);
  EXPECT_FLOAT_EQ(curvy_bonus(0, 0.95f), 1.0f);
}

TEST(CurvyBonus, MaxCurvyAtAlpha06) {
  // PRD: curvy_bonus(255, 0.6) = 0.4 (60% discount).
  EXPECT_NEAR(curvy_bonus(255, 0.6f), 0.4f, 1e-6f);
}

TEST(CurvyBonus, AlphaZeroDisablesBonus) {
  // alpha=0 -> no discount no matter how curvy.
  EXPECT_FLOAT_EQ(curvy_bonus(255, 0.0f), 1.0f);
  EXPECT_FLOAT_EQ(curvy_bonus(128, 0.0f), 1.0f);
}

TEST(CurvyBonus, MonotonicallyDecreasingInByte) {
  // Strictly decreasing in sinuosity_byte for fixed alpha > 0.
  const float alpha = 0.5f;
  float prev = curvy_bonus(0, alpha);
  for (int b = 1; b <= 255; ++b) {
    const float current = curvy_bonus(static_cast<uint8_t>(b), alpha);
    EXPECT_LT(current, prev) << "byte=" << b;
    prev = current;
  }
}

} // namespace
