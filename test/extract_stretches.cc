// Unit tests for the pure helpers in valhalla/mjolnir/stretch_extractor.h.
// better_mc_routing v2 — Issue 02, Slice 1.
//
// Covers seed/grow thresholds, length bounds, split-at-lowest-sinuosity,
// score formula, same-class invariant, straight-blip tolerance. Tests are
// built with synthetic EdgeCandidate vectors so they don't need a tile
// fixture.

#include "mjolnir/stretch_extractor.h"

#include <gtest/gtest.h>

#include <vector>

using valhalla::baldr::RoadClass;
using valhalla::baldr::Use;
using valhalla::mjolnir::blip_stats;
using valhalla::mjolnir::byte_to_raw_sinuosity;
using valhalla::mjolnir::compute_score;
using valhalla::mjolnir::concatenate_shapes;
using valhalla::mjolnir::EdgeCandidate;
using valhalla::mjolnir::emit_with_splits;
using valhalla::mjolnir::extension_eligible;
using valhalla::mjolnir::finalize;
using valhalla::mjolnir::kGrowSinuosityByte;
using valhalla::mjolnir::kMaxStretchKm;
using valhalla::mjolnir::kMaxStraightBlipCount;
using valhalla::mjolnir::kMaxStraightBlipMeters;
using valhalla::mjolnir::kMinStretchKm;
using valhalla::mjolnir::kSeedSinuosityByte;
using valhalla::mjolnir::length_weighted_mean_sinuosity_byte;
using valhalla::mjolnir::passes_emit_filters;
using valhalla::mjolnir::split_at_lowest_sinuosity;
using valhalla::mjolnir::total_length_m;
using valhalla::midgard::PointLL;
using valhalla::sif::ClassMultipliers;

namespace {

// Helper: build an EdgeCandidate with two-point straight shape so all
// helpers see consistent (length_m, shape) pairs.
EdgeCandidate make_edge(uint32_t length_m,
                        uint8_t sinuosity_byte,
                        RoadClass cls = RoadClass::kTertiary,
                        double lat0 = 0.0,
                        double lon0 = 0.0,
                        double lat1 = 0.0,
                        double lon1 = 0.0,
                        uint8_t surface = 1) {  // default paved
  EdgeCandidate e;
  e.length_m = length_m;
  e.sinuosity_byte = sinuosity_byte;
  e.road_class = cls;
  e.use = Use::kRoad;
  e.surface = surface;
  e.roundabout = false;
  e.restricted_access = false;
  if (lat0 == lat1 && lon0 == lon1) {
    e.shape = {PointLL(0.0, 0.0), PointLL(0.001, 0.0)};
  } else {
    e.shape = {PointLL(lon0, lat0), PointLL(lon1, lat1)};
  }
  return e;
}

// ---- Threshold constants ----

TEST(StretchExtractor, SeedAndGrowThresholdsMatchPRD) {
  // PRD seed >= 102 byte (raw ~1.8), grow >= 76 byte (raw ~1.6).
  EXPECT_EQ(kSeedSinuosityByte, 102);
  EXPECT_EQ(kGrowSinuosityByte, 76);
}

TEST(StretchExtractor, LengthBandFromPRD) {
  EXPECT_FLOAT_EQ(kMinStretchKm, 1.0f);
  EXPECT_FLOAT_EQ(kMaxStretchKm, 20.0f);
}

TEST(StretchExtractor, BlipBudgetFromPRD) {
  EXPECT_EQ(kMaxStraightBlipCount, 1u);
  EXPECT_EQ(kMaxStraightBlipMeters, 200u);
}

// ---- length_weighted_mean_sinuosity_byte ----

TEST(StretchExtractor, MeanIsLengthWeighted) {
  // 800 m at byte 200, 200 m at byte 100. Weighted = (800*200 + 200*100)/1000 = 180.
  std::vector<EdgeCandidate> edges{make_edge(800, 200), make_edge(200, 100)};
  EXPECT_FLOAT_EQ(length_weighted_mean_sinuosity_byte(edges), 180.0f);
}

TEST(StretchExtractor, MeanWithZeroLengthIsZero) {
  std::vector<EdgeCandidate> edges{make_edge(0, 200)};
  EXPECT_FLOAT_EQ(length_weighted_mean_sinuosity_byte(edges), 0.0f);
}

// ---- score ----

TEST(StretchExtractor, ScoreIsInZeroOneAndMonotonicInSinuosity) {
  const ClassMultipliers w{};
  const float low = compute_score(50.0f, RoadClass::kTertiary, w);
  const float high = compute_score(200.0f, RoadClass::kTertiary, w);
  EXPECT_GT(high, low);
  EXPECT_GE(low, 0.0f);
  EXPECT_LE(high, 1.0f);
}

TEST(StretchExtractor, ScoreDifferentiatesClasses) {
  const ClassMultipliers w{};
  // Tertiary has class_multiplier 0.85, primary 1.5. So for SAME mean byte,
  // primary should score higher than tertiary.
  const float tertiary = compute_score(150.0f, RoadClass::kTertiary, w);
  const float primary = compute_score(150.0f, RoadClass::kPrimary, w);
  EXPECT_GT(primary, tertiary);
}

TEST(StretchExtractor, ScoreMaxedOutClamped) {
  const ClassMultipliers w{};
  // byte=255, motorway (class_multiplier=5.0=max). sinuosity_norm=1.0,
  // class_term/max=1.0 -> score = 1.0.
  EXPECT_FLOAT_EQ(compute_score(255.0f, RoadClass::kMotorway, w), 1.0f);
}

TEST(StretchExtractor, ScoreZeroForStraight) {
  const ClassMultipliers w{};
  EXPECT_FLOAT_EQ(compute_score(0.0f, RoadClass::kTertiary, w), 0.0f);
}

// ---- byte_to_raw_sinuosity ----

TEST(StretchExtractor, ByteToRawIsInverseOfQuantization) {
  EXPECT_FLOAT_EQ(byte_to_raw_sinuosity(0.0f), 1.0f);   // straight
  EXPECT_FLOAT_EQ(byte_to_raw_sinuosity(127.5f), 2.0f); // mid
  EXPECT_FLOAT_EQ(byte_to_raw_sinuosity(255.0f), 3.0f); // max
}

// ---- length helpers ----

TEST(StretchExtractor, TotalLengthSumsCorrectly) {
  std::vector<EdgeCandidate> edges{make_edge(1234, 100), make_edge(2766, 100)};
  EXPECT_EQ(total_length_m(edges), 4000u);
}

// ---- blip_stats ----

TEST(StretchExtractor, BlipStatsCountsBelowGrow) {
  // grow threshold = 76. Bytes 100 (above), 50 (below, blip), 150 (above), 30 (below, blip).
  std::vector<EdgeCandidate> edges{
      make_edge(300, 100), make_edge(150, 50),
      make_edge(400, 150), make_edge(120, 30)};
  auto s = blip_stats(edges);
  EXPECT_EQ(s.count, 2u);
  EXPECT_EQ(s.total_meters, 270u);
}

// ---- extension_eligible ----

TEST(StretchExtractor, ExtensionRejectsClassChange) {
  EdgeCandidate e = make_edge(500, 100, RoadClass::kSecondary);
  EXPECT_FALSE(extension_eligible(e, RoadClass::kTertiary));
}

TEST(StretchExtractor, ExtensionRejectsRoundabout) {
  EdgeCandidate e = make_edge(500, 200);
  e.roundabout = true;
  EXPECT_FALSE(extension_eligible(e, RoadClass::kTertiary));
}

TEST(StretchExtractor, ExtensionRejectsRestricted) {
  EdgeCandidate e = make_edge(500, 200);
  e.restricted_access = true;
  EXPECT_FALSE(extension_eligible(e, RoadClass::kTertiary));
}

TEST(StretchExtractor, ExtensionAcceptsSameClassEvenIfLowSinuosity) {
  // Eligibility doesn't enforce the sinuosity gate — the caller does
  // (so the caller can apply the BUDGET).
  EdgeCandidate e = make_edge(500, 50);
  EXPECT_TRUE(extension_eligible(e, RoadClass::kTertiary));
}

// ---- split_at_lowest_sinuosity ----

TEST(StretchExtractor, SplitFindsLowestInteriorByte) {
  // 5 edges with interior bytes 150, 50, 180. Split should occur at idx 2
  // (byte 50, the worst interior). Halves = [0, 1] and [3, 4].
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(1000, 150),
                                   make_edge(1000, 50),  make_edge(1000, 180),
                                   make_edge(1000, 200)};
  auto [a, b] = split_at_lowest_sinuosity(edges);
  EXPECT_EQ(a.size(), 2u);
  EXPECT_EQ(b.size(), 2u);
  EXPECT_EQ(a.back().sinuosity_byte, 150u);
  EXPECT_EQ(b.front().sinuosity_byte, 180u);
}

TEST(StretchExtractor, SplitShortStretchReturnsOriginalAndEmpty) {
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(1000, 150)};
  auto [a, b] = split_at_lowest_sinuosity(edges);
  EXPECT_EQ(a.size(), 2u);
  EXPECT_TRUE(b.empty());
}

// ---- emit_with_splits ----

TEST(StretchExtractor, EmitDropsTooShort) {
  // 0.5 km total — below 1 km filter.
  std::vector<EdgeCandidate> edges{make_edge(500, 200)};
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, EmitInBandProducesOneStretch) {
  // 3 km of high-sinuosity edges, no blips.
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(1000, 180),
                                   make_edge(1000, 200)};
  auto out = emit_with_splits(edges);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].length_km, 3.0f);
  EXPECT_GT(out[0].score, 0.0f);
}

TEST(StretchExtractor, EmitTooManyBlipsDrops) {
  // 4 km total, two below-grow blips that exceed the count budget (1).
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(100, 30),
                                   make_edge(1000, 180), make_edge(100, 30),
                                   make_edge(1000, 200)};
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, EmitBlipTooLongDrops) {
  // 4 km total, ONE blip but it's longer than 200 m.
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(300, 30),
                                   make_edge(1000, 200)};
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, EmitLongRunGetsSplit) {
  // 25 km total, no blips. Lowest interior at idx 12 (byte 150).
  std::vector<EdgeCandidate> edges;
  for (int i = 0; i < 25; ++i) {
    uint8_t byte = (i == 12) ? 150 : 200;
    edges.push_back(make_edge(1000, byte));
  }
  auto out = emit_with_splits(edges);
  // After splitting at idx 12, both halves are ~12 km, in band — 2 stretches.
  ASSERT_EQ(out.size(), 2u);
  for (const auto& s : out) {
    EXPECT_GE(s.length_km, kMinStretchKm);
    EXPECT_LE(s.length_km, kMaxStretchKm);
  }
}

TEST(StretchExtractor, EmitDoubleSplitWorks) {
  // 50 km would need multiple splits. After 1st split at lowest, each half is
  // ~25 km — each splits again. Should yield at least 3 stretches in band.
  std::vector<EdgeCandidate> edges;
  for (int i = 0; i < 50; ++i) {
    // Two low-sinuosity break points.
    uint8_t byte = (i == 20 || i == 35) ? 130 : 200;
    edges.push_back(make_edge(1000, byte));
  }
  auto out = emit_with_splits(edges);
  EXPECT_GE(out.size(), 3u);
  for (const auto& s : out) {
    EXPECT_GE(s.length_km, kMinStretchKm);
    EXPECT_LE(s.length_km, kMaxStretchKm);
  }
}

TEST(StretchExtractor, EmitSingleOverlongEdgeDropsNoInfiniteRecursion) {
  // A single curvy edge longer than the 20 km max. It can't be split (split
  // needs >= 3 edges), so it must be dropped — not recursed on forever.
  // Regression: this used to overflow the stack on the Europe tileset.
  std::vector<EdgeCandidate> edges{make_edge(25000, 200)};
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, EmitTwoOverlongEdgesDropsNoInfiniteRecursion) {
  // Two curvy edges, together over the max and each too coarse to split.
  // Same unsplittable-but-overlong case (size < 3) — must drop, not hang.
  std::vector<EdgeCandidate> edges{make_edge(15000, 200), make_edge(15000, 190)};
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, FinalizeRecordsWorstSurface) {
  // Build an in-band run whose edges carry surfaces 1,2,1 -> worst = 2.
  // (In production, growth never mixes surfaces — this guards the metric.)
  std::vector<EdgeCandidate> edges{
      make_edge(1000, 200, RoadClass::kTertiary, 0, 0, 0, 0, 1),
      make_edge(1000, 200, RoadClass::kTertiary, 0, 0, 0, 0, 2),
      make_edge(1000, 200, RoadClass::kTertiary, 0, 0, 0, 0, 1)};
  auto out = emit_with_splits(edges);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].surface, 2);
}

// ---- passes_emit_filters ----

TEST(StretchExtractor, PassesEmitFiltersHappyPath) {
  std::vector<EdgeCandidate> edges{make_edge(2000, 200), make_edge(2000, 180)};
  EXPECT_TRUE(passes_emit_filters(edges));
}

TEST(StretchExtractor, FailsEmitFiltersBelowMin) {
  std::vector<EdgeCandidate> edges{make_edge(800, 200)};
  EXPECT_FALSE(passes_emit_filters(edges));
}

TEST(StretchExtractor, FailsEmitFiltersAboveMax) {
  std::vector<EdgeCandidate> edges;
  for (int i = 0; i < 25; ++i) edges.push_back(make_edge(1000, 200));
  EXPECT_FALSE(passes_emit_filters(edges));
}

// ---- finalize ----

TEST(StretchExtractor, FinalizeProducesRightShapeForOneEdge) {
  std::vector<EdgeCandidate> edges{make_edge(1500, 200, RoadClass::kSecondary)};
  auto s = finalize(edges);
  EXPECT_FLOAT_EQ(s.length_km, 1.5f);
  EXPECT_EQ(s.road_class, RoadClass::kSecondary);
  EXPECT_GT(s.score, 0.0f);
  EXPECT_GT(s.mean_sinuosity_raw, 1.0f);
}

TEST(StretchExtractor, FinalizeRoadClassFromFirstEdge) {
  // Same-class invariant is enforced upstream; finalize just propagates first.
  std::vector<EdgeCandidate> edges{make_edge(1000, 200, RoadClass::kPrimary),
                                   make_edge(1000, 200, RoadClass::kPrimary)};
  auto s = finalize(edges);
  EXPECT_EQ(s.road_class, RoadClass::kPrimary);
}

// ---- concatenate_shapes ----

TEST(StretchExtractor, ConcatenateDedupesJointPoints) {
  EdgeCandidate a, b;
  a.length_m = 1000;
  a.sinuosity_byte = 200;
  a.road_class = RoadClass::kTertiary;
  a.shape = {PointLL(0.0, 0.0), PointLL(0.001, 0.0)};
  b.length_m = 1000;
  b.sinuosity_byte = 200;
  b.road_class = RoadClass::kTertiary;
  b.shape = {PointLL(0.001, 0.0), PointLL(0.002, 0.0)};
  std::vector<EdgeCandidate> edges{a, b};
  auto poly = concatenate_shapes(edges);
  // First edge has 2 points, second has 2 — but the joining point should be
  // de-duplicated, so we expect 3 points total.
  EXPECT_EQ(poly.size(), 3u);
}

} // namespace
