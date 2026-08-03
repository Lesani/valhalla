// Unit tests for the pure helpers in valhalla/sif/scenic_cost_helpers.h.
// better_mc_routing — Issues 06/07/08 introduced the helpers; Issue #18
// replaced the sub-unit multipliers with an admissible (>= 1.0) cost model:
//   - curvy_bonus (discount)        -> straightness_penalty (penalty)
//   - ClassMultipliers renormalized so tertiary = 1.0 is the baseline
//   - toll_multiplier remapped into [1.0, 1.6]

#include "sif/scenic_cost_helpers.h"

#include <gtest/gtest.h>

using valhalla::baldr::RoadClass;
using valhalla::baldr::Surface;
using valhalla::baldr::Use;
using valhalla::sif::ClassMultipliers;
using valhalla::sif::class_multiplier;
using valhalla::sif::highway_class_multiplier;
using valhalla::sif::is_scenic_toll;
using valhalla::sif::kCurvyDetourCap;
using valhalla::sif::kPavedAversion;
using valhalla::sif::kUnpavedRefSpeed;
using valhalla::sif::paved_multiplier;
using valhalla::sif::scaled_class_multipliers;
using valhalla::sif::small_road_multiplier;
using valhalla::sif::straightness_penalty;
using valhalla::sif::toll_multiplier;
using valhalla::sif::track_class_multiplier;

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

// ---- D1: use_highways / use_trails scaled class multipliers ----

TEST(ScaledClassMultipliers, MotorwayAnchorValues) {
  EXPECT_NEAR(scaled_class_multipliers(1.0f, 0.0f).motorway, 1.15f, 1e-5f);
  EXPECT_NEAR(scaled_class_multipliers(0.9f, 0.0f).motorway, 1.36662f, 1e-3f);
  EXPECT_NEAR(scaled_class_multipliers(0.5f, 0.0f).motorway, 3.57184f, 1e-3f);
  EXPECT_NEAR(scaled_class_multipliers(0.1f, 0.0f).motorway, 6.99863f, 1e-3f);
  EXPECT_NEAR(scaled_class_multipliers(0.0f, 0.0f).motorway, 8.0f, 1e-5f);
}

TEST(ScaledClassMultipliers, TrunkAnchorValues) {
  EXPECT_NEAR(scaled_class_multipliers(1.0f, 0.0f).trunk, 1.10f, 1e-5f);
  EXPECT_NEAR(scaled_class_multipliers(0.0f, 0.0f).trunk, 4.5f, 1e-5f);
}

TEST(ScaledClassMultipliers, TrackAnchorValues) {
  EXPECT_NEAR(scaled_class_multipliers(0.1f, 0.0f).track, 6.0f, 1e-5f);
  EXPECT_NEAR(scaled_class_multipliers(0.1f, 0.7f).track, 2.5f, 1e-5f);
  EXPECT_NEAR(scaled_class_multipliers(0.1f, 1.0f).track, 1.0f, 1e-5f);
}

TEST(ScaledClassMultipliers, MonotonicDecreasingInUseHighways) {
  float prev_mw = scaled_class_multipliers(0.0f, 0.0f).motorway;
  float prev_tk = scaled_class_multipliers(0.0f, 0.0f).trunk;
  for (float uh : {0.1f, 0.25f, 0.5f, 0.75f, 0.9f, 1.0f}) {
    const auto w = scaled_class_multipliers(uh, 0.0f);
    EXPECT_LT(w.motorway, prev_mw) << "uh=" << uh;
    EXPECT_LT(w.trunk, prev_tk) << "uh=" << uh;
    prev_mw = w.motorway;
    prev_tk = w.trunk;
  }
}

TEST(ScaledClassMultipliers, MonotonicDecreasingInUseTrails) {
  float prev = scaled_class_multipliers(0.1f, 0.0f).track;
  for (float ut : {0.1f, 0.3f, 0.7f, 0.9f, 1.0f}) {
    const float t = scaled_class_multipliers(0.1f, ut).track;
    EXPECT_LT(t, prev) << "ut=" << ut;
    prev = t;
  }
}

TEST(ScaledClassMultipliers, UnscaledRowsMatchDefaults) {
  const ClassMultipliers def{};
  for (float uh : {0.0f, 0.1f, 0.5f, 0.9f, 1.0f}) {
    for (float ut : {0.0f, 0.7f, 1.0f}) {
      const auto w = scaled_class_multipliers(uh, ut);
      EXPECT_FLOAT_EQ(w.primary, def.primary);
      EXPECT_FLOAT_EQ(w.secondary, def.secondary);
      EXPECT_FLOAT_EQ(w.tertiary, def.tertiary);
      EXPECT_FLOAT_EQ(w.unclassified, def.unclassified);
      EXPECT_FLOAT_EQ(w.residential, def.residential);
      EXPECT_FLOAT_EQ(w.living_street, def.living_street);
      EXPECT_FLOAT_EQ(w.service, def.service);
    }
  }
}

TEST(ScaledClassMultipliers, AllRowsNeverBelowOne) {
  for (float uh : {0.0f, 0.1f, 0.5f, 0.9f, 1.0f}) {
    for (float ut : {0.0f, 0.7f, 1.0f}) {
      const auto w = scaled_class_multipliers(uh, ut);
      for (float v : {w.motorway, w.trunk, w.primary, w.secondary, w.tertiary,
                      w.unclassified, w.residential, w.living_street, w.service, w.track}) {
        EXPECT_GE(v, 1.0f) << "uh=" << uh << " ut=" << ut;
      }
    }
  }
}

// Extends the exhaustive >= 1.0 combined-multiplier grid with the uh/ut axes.
TEST(Admissibility, ScaledCombinedMultiplierNeverBelowOne) {
  const uint8_t sinuosity_bytes[] = {0, 64, 128, 255};
  const float alphas[] = {0.0f, 0.3f, 0.6f, 0.95f};
  const float usts[] = {0.2f, 0.5f, 0.7f};
  const float uhs[] = {0.0f, 0.1f, 0.5f, 0.9f, 1.0f};
  const float uts[] = {0.0f, 0.7f, 1.0f};
  const RoadClass classes[] = {RoadClass::kMotorway,    RoadClass::kTrunk,
                               RoadClass::kPrimary,     RoadClass::kSecondary,
                               RoadClass::kTertiary,    RoadClass::kUnclassified,
                               RoadClass::kResidential, RoadClass::kServiceOther};
  const Use uses[] = {Use::kRoad, Use::kTrack, Use::kLivingStreet};

  for (float uh : uhs) {
    for (float ut : uts) {
      const ClassMultipliers w = scaled_class_multipliers(uh, ut);
      for (uint8_t sin_byte : sinuosity_bytes) {
        for (float alpha : alphas) {
          const float sp = straightness_penalty(sin_byte, alpha);
          for (RoadClass cls : classes) {
            for (Use use : uses) {
              const float cm = class_multiplier(cls, use, w);
              for (bool toll : {false, true}) {
                for (float ust : usts) {
                  const float tm = toll_multiplier(toll, cls, ust);
                  EXPECT_GE(sp * cm * tm, 1.0f)
                      << "uh=" << uh << " ut=" << ut << " sin=" << static_cast<int>(sin_byte)
                      << " alpha=" << alpha << " class=" << static_cast<int>(cls)
                      << " use=" << static_cast<int>(use) << " toll=" << toll << " ust=" << ust;
                }
              }
            }
          }
        }
      }
    }
  }
}

// ---- D1 rev.2: speed-equalized paved_multiplier (profile character) ----

TEST(PavedMultiplier, InertAtOrBelowMidpoint) {
  // At/below use_trails 0.5 the paved penalty is off for every profile, any
  // surface, any speed.
  for (float ut : {0.0f, 0.3f, 0.5f}) {
    for (float speed : {5.0f, 25.0f, 90.0f, 130.0f}) {
      EXPECT_FLOAT_EQ(paved_multiplier(Surface::kPavedSmooth, ut, speed), 1.0f)
          << "ut=" << ut << " speed=" << speed;
      EXPECT_FLOAT_EQ(paved_multiplier(Surface::kPaved, ut, speed), 1.0f)
          << "ut=" << ut << " speed=" << speed;
    }
  }
}

TEST(PavedMultiplier, UnpavedNeverPenalized) {
  // Unpaved edges (Surface >= kCompacted == 3) are never penalized, even at
  // full use_trails and high speed.
  for (Surface s : {Surface::kCompacted, Surface::kDirt, Surface::kGravel, Surface::kPath}) {
    for (float speed : {5.0f, 25.0f, 90.0f, 130.0f}) {
      EXPECT_FLOAT_EQ(paved_multiplier(s, 1.0f, speed), 1.0f)
          << "surface=" << static_cast<int>(s) << " speed=" << speed;
    }
  }
}

TEST(PavedMultiplier, DistanceEqualizationAtFullTrails) {
  // ut=1.0: the paved penalty equalizes per-km cost to a kUnpavedRefSpeed
  // (25 km/h) gravel edge times kPavedAversion, so speed cancels in per-km
  // terms: pm(speed) == (speed / 25) * 2.5 for speed >= 25. (Patch 0018
  // raised kPavedAversion 1.5 -> 2.5: measured on the surfaced Sweden A/B,
  // 1.5 left adventure at 68.5% unpaved vs 80.7% at 2.5; gains saturate
  // above 2.5 — see docs/designs/design-surface-inference.md.)
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 100.0f), 10.0f, 1e-5f);
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 50.0f), 5.0f, 1e-5f);
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 25.0f), 2.5f, 1e-5f);
  // Per-km cost (pm / speed) is a constant kPavedAversion / kUnpavedRefSpeed.
  const float per_km = kPavedAversion / kUnpavedRefSpeed;
  for (float speed : {25.0f, 50.0f, 100.0f}) {
    EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, speed) / speed, per_km, 1e-6f)
        << "speed=" << speed;
  }
}

TEST(PavedMultiplier, SlowPavedFloorsAtAversion) {
  // ut=1.0, speed below kUnpavedRefSpeed floors at kPavedAversion (the
  // max(1, speed/ref) guard): a slow paved lane never costs less than 2.5x.
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 10.0f), kPavedAversion, 1e-5f);
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 5.0f), kPavedAversion, 1e-5f);
  EXPECT_NEAR(paved_multiplier(Surface::kPaved, 1.0f, 24.9f), kPavedAversion, 1e-5f);
}

TEST(PavedMultiplier, MonotonicIncreasingAboveMidpoint) {
  // Fixed speed 80, strictly increasing in use_trails on (0.5, 1.0].
  const float speed = 80.0f;
  float prev = paved_multiplier(Surface::kPaved, 0.5f, speed); // 1.0 (inert)
  for (float ut : {0.55f, 0.6f, 0.75f, 0.9f, 1.0f}) {
    const float current = paved_multiplier(Surface::kPaved, ut, speed);
    EXPECT_GT(current, prev) << "ut=" << ut;
    prev = current;
  }
}

TEST(PavedMultiplier, NeverBelowOne) {
  const Surface surfaces[] = {Surface::kPavedSmooth, Surface::kPaved,   Surface::kPavedRough,
                              Surface::kCompacted,   Surface::kDirt,    Surface::kGravel,
                              Surface::kPath};
  for (Surface s : surfaces) {
    for (float ut : {0.0f, 0.3f, 0.5f, 0.7f, 1.0f}) {
      for (float speed : {5.0f, 25.0f, 50.0f, 100.0f, 130.0f}) {
        EXPECT_GE(paved_multiplier(s, ut, speed), 1.0f)
            << "surface=" << static_cast<int>(s) << " ut=" << ut << " speed=" << speed;
      }
    }
  }
}

// ---- D2: use_small_roads scaled class multipliers (profile character) ----

TEST(SmallRoadMultipliers, TwistyHunterAnchors) {
  // usr=0.8 (design D3 twisty_hunter anchors).
  const auto w = scaled_class_multipliers(0.1f, 0.0f, 0.8f);
  EXPECT_NEAR(w.residential, 1.13f, 1e-4f);
  EXPECT_NEAR(w.living_street, 1.27f, 1e-4f);
  EXPECT_NEAR(w.service, 1.388f, 1e-4f);
  EXPECT_NEAR(w.unclassified, 1.012f, 1e-4f);
}

TEST(SmallRoadMultipliers, UsrZeroMatchesDefaults) {
  const ClassMultipliers def{};
  const auto w = scaled_class_multipliers(0.1f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(w.residential, def.residential);
  EXPECT_FLOAT_EQ(w.living_street, def.living_street);
  EXPECT_FLOAT_EQ(w.service, def.service);
  EXPECT_FLOAT_EQ(w.unclassified, def.unclassified);
}

TEST(SmallRoadMultipliers, UsrOneCollapsesToOne) {
  const auto w = scaled_class_multipliers(0.1f, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(w.residential, 1.0f);
  EXPECT_FLOAT_EQ(w.living_street, 1.0f);
  EXPECT_FLOAT_EQ(w.service, 1.0f);
  EXPECT_FLOAT_EQ(w.unclassified, 1.0f);
  // The pure helper collapses any small-road row default to 1.0 at usr=1.
  EXPECT_FLOAT_EQ(small_road_multiplier(2.94f, 1.0f), 1.0f);
}

TEST(SmallRoadMultipliers, MonotonicDecreasingInUsr) {
  float prev_res = scaled_class_multipliers(0.1f, 0.0f, 0.0f).residential;
  float prev_ls = scaled_class_multipliers(0.1f, 0.0f, 0.0f).living_street;
  float prev_svc = scaled_class_multipliers(0.1f, 0.0f, 0.0f).service;
  float prev_unc = scaled_class_multipliers(0.1f, 0.0f, 0.0f).unclassified;
  for (float usr : {0.2f, 0.4f, 0.6f, 0.8f, 1.0f}) {
    const auto w = scaled_class_multipliers(0.1f, 0.0f, usr);
    EXPECT_LE(w.residential, prev_res) << "usr=" << usr;
    EXPECT_LE(w.living_street, prev_ls) << "usr=" << usr;
    EXPECT_LE(w.service, prev_svc) << "usr=" << usr;
    EXPECT_LE(w.unclassified, prev_unc) << "usr=" << usr;
    prev_res = w.residential;
    prev_ls = w.living_street;
    prev_svc = w.service;
    prev_unc = w.unclassified;
  }
}

TEST(SmallRoadMultipliers, OtherRowsUnaffected) {
  // motorway/trunk/primary/secondary/tertiary/track do not move with usr when
  // use_highways and use_trails are held fixed.
  const auto base = scaled_class_multipliers(0.3f, 0.4f, 0.0f);
  for (float usr : {0.0f, 0.5f, 1.0f}) {
    const auto w = scaled_class_multipliers(0.3f, 0.4f, usr);
    EXPECT_FLOAT_EQ(w.motorway, base.motorway) << "usr=" << usr;
    EXPECT_FLOAT_EQ(w.trunk, base.trunk) << "usr=" << usr;
    EXPECT_FLOAT_EQ(w.primary, base.primary) << "usr=" << usr;
    EXPECT_FLOAT_EQ(w.secondary, base.secondary) << "usr=" << usr;
    EXPECT_FLOAT_EQ(w.tertiary, base.tertiary) << "usr=" << usr;
    EXPECT_FLOAT_EQ(w.track, base.track) << "usr=" << usr;
  }
}

TEST(SmallRoadMultipliers, AllRowsNeverBelowOne) {
  for (float uh : {0.0f, 0.1f, 0.5f, 1.0f}) {
    for (float ut : {0.0f, 0.7f, 1.0f}) {
      for (float usr : {0.0f, 0.3f, 0.6f, 1.0f}) {
        const auto w = scaled_class_multipliers(uh, ut, usr);
        for (float v : {w.motorway, w.trunk, w.primary, w.secondary, w.tertiary, w.unclassified,
                        w.residential, w.living_street, w.service, w.track}) {
          EXPECT_GE(v, 1.0f) << "uh=" << uh << " ut=" << ut << " usr=" << usr;
        }
      }
    }
  }
}

// ---- Admissibility with the two new factors ----

TEST(Admissibility, CombinedWithPavedAndSmallRoadsNeverBelowOne) {
  // Extend the exhaustive grid with usr (scaled table) and the pm factor over
  // paved + unpaved surfaces and speeds. sp * cm * tm * pm >= 1.0 everywhere.
  const uint8_t sinuosity_bytes[] = {0, 64, 128, 255};
  const float alphas[] = {0.0f, 0.3f, 0.6f, 0.95f};
  const float usts[] = {0.2f, 0.5f, 0.7f};
  const float uhs[] = {0.0f, 0.1f, 1.0f};
  const float uts[] = {0.0f, 0.7f, 1.0f};
  const float usrs[] = {0.0f, 0.6f, 1.0f};
  const float speeds[] = {5.0f, 25.0f, 100.0f, 130.0f};
  const RoadClass classes[] = {RoadClass::kMotorway,    RoadClass::kTrunk,
                               RoadClass::kPrimary,     RoadClass::kSecondary,
                               RoadClass::kTertiary,    RoadClass::kUnclassified,
                               RoadClass::kResidential, RoadClass::kServiceOther};
  const Use uses[] = {Use::kRoad, Use::kTrack, Use::kLivingStreet};
  const Surface surfaces[] = {Surface::kPavedSmooth, Surface::kPaved, Surface::kGravel,
                              Surface::kPath};

  for (float uh : uhs) {
    for (float ut : uts) {
      for (float usr : usrs) {
        const ClassMultipliers w = scaled_class_multipliers(uh, ut, usr);
        for (uint8_t sin_byte : sinuosity_bytes) {
          for (float alpha : alphas) {
            const float sp = straightness_penalty(sin_byte, alpha);
            for (RoadClass cls : classes) {
              for (Use use : uses) {
                const float cm = class_multiplier(cls, use, w);
                for (bool toll : {false, true}) {
                  for (float ust : usts) {
                    const float tm = toll_multiplier(toll, cls, ust);
                    for (Surface s : surfaces) {
                      for (float speed : speeds) {
                        const float pm = paved_multiplier(s, ut, speed);
                        EXPECT_GE(sp * cm * tm * pm, 1.0f)
                            << "uh=" << uh << " ut=" << ut << " usr=" << usr
                            << " sin=" << static_cast<int>(sin_byte) << " alpha=" << alpha
                            << " class=" << static_cast<int>(cls) << " use=" << static_cast<int>(use)
                            << " toll=" << toll << " ust=" << ust
                            << " surface=" << static_cast<int>(s) << " speed=" << speed;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

// ---- A2: the constants must beat the paved/gravel speed gap ----

TEST(AdventureProfile, GravelBeatsPavedPrimaryPerKm) {
  // Adventure knobs (uh=0.1, ut=1.0, usr=0.6). On the Karlskrona corridor the
  // fastest route rides a paved primary at 100 km/h while the scenic corridor
  // is 25 km/h gravel (an unclassified way). The stacked class + paved factors
  // must make the paved primary MORE expensive PER KM than the gravel so the
  // adventure profile leaves the asphalt. This pins the constants against
  // future retuning below the 4x speed ratio.
  const ClassMultipliers w = scaled_class_multipliers(0.1f, 1.0f, 0.6f);

  const float cm_primary = class_multiplier(RoadClass::kPrimary, Use::kRoad, w);
  const float cm_gravel = class_multiplier(RoadClass::kUnclassified, Use::kRoad, w);
  const float pm_paved = paved_multiplier(Surface::kPaved, 1.0f, 100.0f);
  const float pm_gravel = paved_multiplier(Surface::kGravel, 1.0f, 25.0f);

  const float paved_per_km = cm_primary * pm_paved / 100.0f;
  const float gravel_per_km = cm_gravel * pm_gravel / 25.0f;
  EXPECT_GT(paved_per_km, gravel_per_km)
      << "paved_per_km=" << paved_per_km << " gravel_per_km=" << gravel_per_km;
}

} // namespace
