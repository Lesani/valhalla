// Unit tests for the pure helpers in valhalla/sif/scenic_cost_helpers.h.
// better_mc_routing — Issues 06/07/08 introduced the helpers; Issue #18
// replaced the sub-unit multipliers with an admissible (>= 1.0) cost model:
//   - curvy_bonus (discount)        -> straightness_penalty (penalty)
//   - ClassMultipliers renormalized so tertiary = 1.0 is the baseline
//   - toll_multiplier remapped into [1.0, 1.6]

#include "sif/scenic_cost_helpers.h"

#include <gtest/gtest.h>

using valhalla::baldr::RoadClass;
using valhalla::baldr::Use;
using valhalla::sif::ClassMultipliers;
using valhalla::sif::class_multiplier;
using valhalla::sif::is_scenic_toll;
using valhalla::sif::kCurvyDetourCap;
using valhalla::sif::straightness_penalty;
using valhalla::sif::toll_multiplier;

namespace {

// ---- straightness_penalty (Issue 06, admissible since Issue #18) ----

TEST(StraightnessPenalty, MaxCurvyEdgePaysBaseCost) {
  // The curviest edge pays exactly its base cost regardless of alpha.
  EXPECT_FLOAT_EQ(straightness_penalty(255, 0.0f), 1.0f);
  EXPECT_FLOAT_EQ(straightness_penalty(255, 0.6f), 1.0f);
  EXPECT_FLOAT_EQ(straightness_penalty(255, 0.95f), 1.0f);
}

TEST(StraightnessPenalty, DeadStraightEdgePaysFullPenalty) {
  // Dead straight edge -> 1 + alpha * kCurvyDetourCap.
  EXPECT_NEAR(straightness_penalty(0, 0.6f), 1.0f + 0.6f * kCurvyDetourCap, 1e-6f);
  EXPECT_NEAR(straightness_penalty(0, 0.6f), 1.9f, 1e-6f);
  EXPECT_NEAR(straightness_penalty(0, 0.95f), 1.0f + 0.95f * kCurvyDetourCap, 1e-6f);
}

TEST(StraightnessPenalty, AlphaZeroDisablesPenalty) {
  EXPECT_FLOAT_EQ(straightness_penalty(0, 0.0f), 1.0f);
  EXPECT_FLOAT_EQ(straightness_penalty(128, 0.0f), 1.0f);
}

TEST(StraightnessPenalty, MonotonicallyDecreasingInByte) {
  // v3: non-increasing in the curve-density byte for fixed alpha > 0 (the
  // curvier, the cheaper), STRICTLY decreasing up to kCurveDensityFullByte and
  // then flat at base cost (1.0) for any byte >= it (fully-curvy plateau).
  const float alpha = 0.5f;
  const int full = static_cast<int>(valhalla::sif::kCurveDensityFullByte);
  float prev = straightness_penalty(0, alpha);
  for (int b = 1; b <= 255; ++b) {
    const float current = straightness_penalty(static_cast<uint8_t>(b), alpha);
    if (b <= full) {
      EXPECT_LT(current, prev) << "byte=" << b;
    } else {
      EXPECT_FLOAT_EQ(current, 1.0f) << "byte=" << b;
    }
    prev = current;
  }
}

TEST(StraightnessPenalty, NeverBelowOne) {
  for (float alpha : {0.0f, 0.3f, 0.6f, 0.95f}) {
    for (int b = 0; b <= 255; ++b) {
      EXPECT_GE(straightness_penalty(static_cast<uint8_t>(b), alpha), 1.0f)
          << "alpha=" << alpha << " byte=" << b;
    }
  }
}

// ---- class_multiplier (Issue 07, renormalized by Issue #18) ----

TEST(ClassMultiplier, RenormalizedValues) {
  // Old v1 table divided by 0.85 so tertiary is the 1.0 baseline.
  const ClassMultipliers w{};
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kMotorway, Use::kRoad, w), 5.88f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kTrunk, Use::kRoad, w), 3.53f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kPrimary, Use::kRoad, w), 1.76f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kSecondary, Use::kRoad, w), 1.18f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kTertiary, Use::kRoad, w), 1.00f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kUnclassified, Use::kRoad, w), 1.06f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kResidential, Use::kRoad, w), 1.65f);
}

TEST(ClassMultiplier, UseOverridesRoadClass) {
  // kTrack and kLivingStreet are Use sub-categories that win over the
  // RoadClass — a residential-tagged way that's actually a forest track
  // pays the track price, not residential.
  const ClassMultipliers w{};
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kResidential, Use::kTrack, w), 5.88f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kTertiary, Use::kLivingStreet, w), 2.35f);
  EXPECT_FLOAT_EQ(class_multiplier(RoadClass::kServiceOther, Use::kRoad, w), 2.94f);
}

TEST(ClassMultiplier, CalimotoFixDirection) {
  // The Calimoto fix under the admissible model at alpha = 0.6
  // (length-only proxy: class_mult * straightness_penalty):
  //   curvy tertiary       = 1.00 * 1.0 = 1.00   (global optimum)
  //   straight tertiary    = 1.00 * 1.9 = 1.90
  //   curvy residential    = 1.65 * 1.0 = 1.65
  //   straight residential = 1.65 * 1.9 = 3.135  (most expensive)
  // Pin the ordering relations the weights are designed around.
  const ClassMultipliers w{};
  const float alpha = 0.6f;

  const float straight_tertiary =
      class_multiplier(RoadClass::kTertiary, Use::kRoad, w) * straightness_penalty(0, alpha);
  const float curvy_tertiary =
      class_multiplier(RoadClass::kTertiary, Use::kRoad, w) * straightness_penalty(255, alpha);
  const float straight_residential =
      class_multiplier(RoadClass::kResidential, Use::kRoad, w) * straightness_penalty(0, alpha);
  const float curvy_residential =
      class_multiplier(RoadClass::kResidential, Use::kRoad, w) * straightness_penalty(255, alpha);

  // Curvy tertiary is the global optimum.
  EXPECT_LT(curvy_tertiary, curvy_residential);
  EXPECT_LT(curvy_tertiary, straight_tertiary);
  // A straight village street is the most expensive of the four.
  EXPECT_GT(straight_residential, straight_tertiary);
  EXPECT_GT(straight_residential, curvy_residential);
  // Everything >= 1.0 (admissibility).
  for (float v : {straight_tertiary, curvy_tertiary, straight_residential, curvy_residential}) {
    EXPECT_GE(v, 1.0f);
  }
}

// ---- is_scenic_toll + toll_multiplier (Issue 08, remapped by Issue #18) ----

TEST(ScenicToll, IsScenicTollClassHeuristic) {
  // Tolled secondary/tertiary/unclassified are scenic (mountain passes);
  // tolled motorway/trunk/primary are road tolls (highway tolls/tunnels).
  EXPECT_TRUE(is_scenic_toll(true, RoadClass::kTertiary));
  EXPECT_TRUE(is_scenic_toll(true, RoadClass::kSecondary));
  EXPECT_TRUE(is_scenic_toll(true, RoadClass::kUnclassified));
  EXPECT_FALSE(is_scenic_toll(true, RoadClass::kMotorway));
  EXPECT_FALSE(is_scenic_toll(true, RoadClass::kTrunk));
  EXPECT_FALSE(is_scenic_toll(true, RoadClass::kPrimary));
  // Untolled edges are never scenic tolls regardless of class.
  EXPECT_FALSE(is_scenic_toll(false, RoadClass::kTertiary));
  EXPECT_FALSE(is_scenic_toll(false, RoadClass::kMotorway));
}

TEST(ScenicToll, MultiplierScalesWithUseScenicTolls) {
  // Scenic-toll mapping into [1.0, 1.6]: full appetite (0.7) is NEUTRAL,
  // not a discount — the price of admissibility (Issue #18).
  const float low_pref = toll_multiplier(true, RoadClass::kTertiary, 0.2f);
  const float mid_pref = toll_multiplier(true, RoadClass::kTertiary, 0.5f);
  const float high_pref = toll_multiplier(true, RoadClass::kTertiary, 0.7f);
  EXPECT_NEAR(low_pref, 1.6f, 1e-5f);
  EXPECT_NEAR(mid_pref, 1.24f, 1e-5f);
  EXPECT_NEAR(high_pref, 1.0f, 1e-5f);
  EXPECT_LT(high_pref, low_pref);
}

TEST(ScenicToll, RoadTollFixedAvoidRegardlessOfPreference) {
  // Road-toll multiplier is FIXED at 1.6 no matter what scenic-pref the
  // user sets — can't accidentally unlock the A10.
  EXPECT_NEAR(toll_multiplier(true, RoadClass::kMotorway, 0.2f), 1.6f, 1e-5f);
  EXPECT_NEAR(toll_multiplier(true, RoadClass::kMotorway, 0.5f), 1.6f, 1e-5f);
  EXPECT_NEAR(toll_multiplier(true, RoadClass::kMotorway, 0.7f), 1.6f, 1e-5f);
  EXPECT_NEAR(toll_multiplier(true, RoadClass::kTrunk, 0.7f), 1.6f, 1e-5f);
  EXPECT_NEAR(toll_multiplier(true, RoadClass::kPrimary, 0.7f), 1.6f, 1e-5f);
}

TEST(ScenicToll, UntolledEdgeIsNeutral) {
  EXPECT_FLOAT_EQ(toll_multiplier(false, RoadClass::kMotorway, 0.5f), 1.0f);
  EXPECT_FLOAT_EQ(toll_multiplier(false, RoadClass::kTertiary, 0.2f), 1.0f);
}

// ---- Issue #18 invariant: the combined multiplier is admissible ----

TEST(Admissibility, CombinedMultiplierNeverBelowOne) {
  // MotorcycleCurvyCost::EdgeCost = MotorcycleCost::EdgeCost * sp * cm * tm.
  // If sp * cm * tm >= 1.0 on every edge then the curvy cost dominates the
  // base motorcycle cost everywhere, i.e. EdgeCost(curvy) >= EdgeCost(motorcycle),
  // and any heuristic admissible for the base costing remains admissible for
  // motorcycle_curvy. Sweep the full grid of inputs the three helpers see.
  const ClassMultipliers w{};

  const uint8_t sinuosity_bytes[] = {0, 64, 128, 255};
  const float alphas[] = {0.0f, 0.3f, 0.6f, 0.95f};
  const float usts[] = {0.2f, 0.5f, 0.7f};
  const RoadClass classes[] = {RoadClass::kMotorway,    RoadClass::kTrunk,
                               RoadClass::kPrimary,     RoadClass::kSecondary,
                               RoadClass::kTertiary,    RoadClass::kUnclassified,
                               RoadClass::kResidential, RoadClass::kServiceOther};
  const Use uses[] = {Use::kRoad, Use::kTrack, Use::kLivingStreet};

  for (uint8_t sin_byte : sinuosity_bytes) {
    for (float alpha : alphas) {
      const float sp = straightness_penalty(sin_byte, alpha);
      for (RoadClass cls : classes) {
        for (Use use : uses) {
          const float cm = class_multiplier(cls, use, w);
          for (bool toll : {false, true}) {
            for (float ust : usts) {
              const float tm = toll_multiplier(toll, cls, ust);
              const float combined = sp * cm * tm;
              EXPECT_GE(combined, 1.0f)
                  << "sin_byte=" << static_cast<int>(sin_byte) << " alpha=" << alpha
                  << " class=" << static_cast<int>(cls) << " use=" << static_cast<int>(use)
                  << " toll=" << toll << " ust=" << ust << " (sp=" << sp << " cm=" << cm
                  << " tm=" << tm << ")";
            }
          }
        }
      }
    }
  }
}

} // namespace
