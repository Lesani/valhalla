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
using valhalla::mjolnir::compute_shape_curve;
using valhalla::mjolnir::curve_circumradius;
using valhalla::mjolnir::curve_density;
using valhalla::mjolnir::curve_weight_for_radius;
using valhalla::sif::ClassMultipliers;
// WS-C1 (patch 0022): radius-bucket histogram + interruption data.
using valhalla::mjolnir::bucket_shares_q8;
using valhalla::mjolnir::junctions_per_km;
using valhalla::mjolnir::kBucketCount;
using valhalla::mjolnir::kBucketResampleStepM;
using valhalla::mjolnir::length_weighted_mean_density;
using valhalla::mjolnir::resample_by_arclength;
using valhalla::mjolnir::shape_bucket_lengths;

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
  // v3: synthesize a curve density from the legacy sinuosity byte so these
  // synthetic edges read as curvy (byte >= grow threshold -> density 0.6) or
  // straight (below -> 0, and the whole edge counts as a straight run).
  e.curvy_m = sinuosity_byte >= 76 ? 0.6f * static_cast<float>(length_m) : 0.0f;
  e.max_straight_m = sinuosity_byte < 76 ? static_cast<float>(length_m) : 0.0f;
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
  EXPECT_FLOAT_EQ(kMinStretchKm, 3.0f); // v3: highlights are substantial roads
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

TEST(StretchExtractor, EmitMostlyStraightRunDropsBelowDensityFloor) {
  // v3: a run that is mostly straight (one short curvy edge among long straights)
  // has curve density below kEmitCurveDensity -> dropped. (Replaces the old
  // blip-count tests; the curve-density floor subsumes the blip budget.)
  std::vector<EdgeCandidate> edges{make_edge(200, 200), make_edge(2000, 30),
                                   make_edge(2000, 30)};
  // density = 0.6*200 / 4200 = 0.029 < 0.15 floor.
  EXPECT_TRUE(emit_with_splits(edges).empty());
}

TEST(StretchExtractor, EmitCurvyRunWithShortStraightStillEmits) {
  // v3: a genuinely curvy run keeps a short straight without being dropped
  // (density stays above the floor) — the hairpins-with-straights case.
  std::vector<EdgeCandidate> edges{make_edge(2000, 200), make_edge(300, 30),
                                   make_edge(2000, 200)};
  // 4.3 km (>= 3 km floor); density = 0.6*4000 / 4300 = 0.56 >> floor.
  auto out = emit_with_splits(edges);
  ASSERT_EQ(out.size(), 1u);
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

// ---- curve metric (Menger radius-binning) ----

TEST(StretchExtractor, CurveWeightBins) {
  EXPECT_FLOAT_EQ(curve_weight_for_radius(20.0), 2.0f);
  EXPECT_FLOAT_EQ(curve_weight_for_radius(45.0), 1.6f);
  EXPECT_FLOAT_EQ(curve_weight_for_radius(80.0), 1.3f);
  EXPECT_FLOAT_EQ(curve_weight_for_radius(150.0), 1.0f);
  EXPECT_FLOAT_EQ(curve_weight_for_radius(500.0), 0.0f);
}

TEST(StretchExtractor, CircumradiusCollinearIsInfinite) {
  // Degenerate triangle (a straight line) -> infinite radius.
  EXPECT_TRUE(std::isinf(curve_circumradius(100.0, 100.0, 200.0)));
}

TEST(StretchExtractor, CurveMetricStraightIsZero) {
  std::vector<PointLL> line;
  for (int i = 0; i < 12; ++i) {
    line.emplace_back(11.0 + i * 0.001, 47.0); // due east, perfectly straight
  }
  auto c = compute_shape_curve(line);
  EXPECT_GT(c.length_m, 0.0f);
  EXPECT_NEAR(c.curvy_m, 0.0f, 1.0f);
  EXPECT_LT(curve_density(c), 0.01f);
  EXPECT_GT(c.max_straight_m, 0.0f);
}

TEST(StretchExtractor, CurveMetricTightArcScoresHigh) {
  // Points on a ~50 m radius half-circle -> circumradius ~50 m -> weight 1.6.
  const double R = 50.0, lat0 = 47.0, lon0 = 11.0;
  const double mlat = 111320.0, mlon = 111320.0 * std::cos(lat0 * 3.14159265358979 / 180.0);
  std::vector<PointLL> arc;
  for (int i = 0; i <= 24; ++i) {
    const double th = 3.14159265358979 * i / 24.0;
    arc.emplace_back(lon0 + (R / mlon) * std::sin(th),
                     lat0 + (R / mlat) * (1.0 - std::cos(th)));
  }
  auto c = compute_shape_curve(arc);
  EXPECT_GT(curve_density(c), 1.0f);     // tight curve -> high density
  EXPECT_NEAR(c.max_straight_m, 0.0f, 1.0f);
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
  // v3: score is the curve density (~0.6 for a fully-curvy synthetic edge);
  // mean_sinuosity_raw is repurposed to carry the same density into the QA dump.
  EXPECT_GT(s.score, 0.0f);
  EXPECT_FLOAT_EQ(s.mean_sinuosity_raw, s.score);
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

// ---- WS-C1 patch 0022: 20 m resample + radius-bucket histogram ----

namespace {

constexpr double kPi = 3.14159265358979;

// Points on a circular arc of radius R centred so the arc starts at
// (lat0, lon0) and sweeps `sweep_rad`. `n` segments.
std::vector<PointLL> make_arc(double radius_m, double sweep_rad, int n) {
  const double lat0 = 47.0, lon0 = 11.0;
  const double mlat = 111320.0, mlon = 111320.0 * std::cos(lat0 * kPi / 180.0);
  std::vector<PointLL> arc;
  for (int i = 0; i <= n; ++i) {
    const double th = sweep_rad * i / n;
    arc.emplace_back(lon0 + (radius_m / mlon) * std::sin(th),
                     lat0 + (radius_m / mlat) * (1.0 - std::cos(th)));
  }
  return arc;
}

// Sum of the four CURVE bins (index 4 is the straight sink).
float classified_length(const std::array<float, kBucketCount + 1>& bins) {
  float sum = 0.0f;
  for (size_t b = 0; b < kBucketCount; ++b) sum += bins[b];
  return sum;
}

} // namespace

TEST(StretchExtractor, ResampleStepsAtFixedArcLength) {
  // One straight 1 km-ish segment resampled at 20 m: points land at
  // 0, 20, 40 ... plus the preserved endpoint.
  const std::vector<PointLL> line{PointLL(11.0, 47.0), PointLL(11.0132, 47.0)};
  const double len = line[0].Distance(line[1]);
  auto out = resample_by_arclength(line, kBucketResampleStepM);
  const size_t expected = static_cast<size_t>(std::floor(len / kBucketResampleStepM)) + 1;
  // +1 more if the final leftover is a real distance (endpoint preserved).
  EXPECT_GE(out.size(), expected);
  EXPECT_LE(out.size(), expected + 1);
  // Every interior gap is the step, to within a centimetre.
  for (size_t i = 0; i + 2 < out.size(); ++i) {
    EXPECT_NEAR(out[i].Distance(out[i + 1]), kBucketResampleStepM, 0.01);
  }
}

TEST(StretchExtractor, ResamplePreservesEndpoints) {
  auto arc = make_arc(120.0, kPi, 7); // coarse: gaps far wider than 20 m
  auto out = resample_by_arclength(arc, kBucketResampleStepM);
  ASSERT_GE(out.size(), 2u);
  EXPECT_DOUBLE_EQ(out.front().lat(), arc.front().lat());
  EXPECT_DOUBLE_EQ(out.front().lng(), arc.front().lng());
  EXPECT_DOUBLE_EQ(out.back().lat(), arc.back().lat());
  EXPECT_DOUBLE_EQ(out.back().lng(), arc.back().lng());
  // A coarse shape gains points; the resample must not shrink it here.
  EXPECT_GT(out.size(), arc.size());
}

TEST(StretchExtractor, ResampleSkipsDegenerateShapes) {
  // Fewer than two points: returned untouched.
  EXPECT_TRUE(resample_by_arclength({}, kBucketResampleStepM).empty());
  const std::vector<PointLL> one{PointLL(11.0, 47.0)};
  EXPECT_EQ(resample_by_arclength(one, kBucketResampleStepM).size(), 1u);
  // Two coincident points: the zero-length segment is skipped and the
  // endpoint dedupes against the start, leaving a single point.
  const std::vector<PointLL> dup{PointLL(11.0, 47.0), PointLL(11.0, 47.0)};
  EXPECT_EQ(resample_by_arclength(dup, kBucketResampleStepM).size(), 1u);
  // A non-positive step means "raw triples" -- the shape passes through.
  auto arc = make_arc(50.0, kPi, 24);
  EXPECT_EQ(resample_by_arclength(arc, 0.0).size(), arc.size());
}

TEST(StretchExtractor, BucketLengthsBinSyntheticCircle) {
  // A 50 m-radius arc: every resampled triple sits on the same circle, so the
  // Menger radius is 50 m and the length belongs to bin 1 (30-60 m).
  auto tight = shape_bucket_lengths(make_arc(50.0, kPi, 24));
  EXPECT_GT(tight[1], 0.0f);
  EXPECT_GT(tight[1], tight[0] + tight[2] + tight[3] + tight[4]);
  // A 500 m-radius arc is a sweeper past the 175 m bin edge: it all lands in
  // the straight sink and contributes NO classified curvature.
  auto wide = shape_bucket_lengths(make_arc(500.0, kPi / 2.0, 40));
  EXPECT_GT(wide[4], 0.0f);
  EXPECT_NEAR(classified_length(wide), 0.0f, 1.0f);
}

TEST(StretchExtractor, ResampleChangesSharesVsRawTriples) {
  // The reason the 20 m resample is MANDATORY (ws_c1_gate.md section 1): a
  // dead-straight road sampled every ~6 m with 1e-6-degree (~0.11 m)
  // coordinate wobble has Menger radii of ~d^2/(2h) ~ 160 m, so the RAW
  // triples file real straight into the 100-175 m curve bin. The resample
  // must wash that out.
  std::vector<PointLL> noisy;
  for (int i = 0; i < 60; ++i) {
    const double lon = 11.0 + i * 0.00008;        // ~6 m steps at lat 47
    const double lat = 47.0 + (i % 2 ? 1e-6 : 0); // quantization wobble
    noisy.emplace_back(lon, lat);
  }
  const auto raw = shape_bucket_lengths(noisy, 0.0); // 0 == no resample
  const auto resampled = shape_bucket_lengths(noisy, kBucketResampleStepM);
  // Raw triples fabricate curvature over most of the shape...
  EXPECT_GT(classified_length(raw), 0.5f * raw[4]);
  EXPECT_GT(raw[3], 0.0f);
  // ...and the 20 m resample removes essentially all of it.
  EXPECT_LT(classified_length(resampled), 0.05f * classified_length(raw));
}

TEST(StretchExtractor, BucketSharesQ8SumsAndStraightIsAllZero) {
  // 100/200/300/400 m of classified curvature (plus 1000 m of straight, which
  // must NOT dilute the shares) -> 25.5/51/76.5/102 of 255.
  EdgeCandidate e = make_edge(2000, 200);
  e.bucket_len_m = {100.0f, 200.0f, 300.0f, 400.0f, 1000.0f};
  std::vector<EdgeCandidate> edges{e};
  auto shares = bucket_shares_q8(edges);
  int sum = 0;
  for (size_t b = 0; b < kBucketCount; ++b) sum += shares[b];
  EXPECT_NEAR(sum, 255, 2);
  EXPECT_LT(shares[0], shares[1]);
  EXPECT_LT(shares[1], shares[2]);
  EXPECT_LT(shares[2], shares[3]);
  EXPECT_NEAR(shares[3], 102, 1);
  // A perfectly straight run has no classified curvature at all: the
  // all-zero sentinel, which consumers read as the neutral 1.0 factor.
  EdgeCandidate straight = make_edge(2000, 200);
  straight.bucket_len_m = {0.0f, 0.0f, 0.0f, 0.0f, 2000.0f};
  std::vector<EdgeCandidate> flat{straight};
  auto zero = bucket_shares_q8(flat);
  for (size_t b = 0; b < kBucketCount; ++b) EXPECT_EQ(zero[b], 0);
}

TEST(StretchExtractor, MeanDensityIsLengthWeighted) {
  // 800 m at density 10, 200 m at density 5 -> (8000 + 1000) / 1000 = 9.
  EdgeCandidate a = make_edge(800, 200);
  a.density = 10;
  EdgeCandidate b = make_edge(200, 200);
  b.density = 5;
  std::vector<EdgeCandidate> edges{a, b};
  EXPECT_FLOAT_EQ(length_weighted_mean_density(edges), 9.0f);
  // Defensive: a zero-length run reports 0 rather than dividing by zero.
  EdgeCandidate empty_edge = make_edge(0, 200);
  empty_edge.density = 12;
  std::vector<EdgeCandidate> empty_run{empty_edge};
  EXPECT_FLOAT_EQ(length_weighted_mean_density(empty_run), 0.0f);
}

TEST(StretchExtractor, JunctionsPerKmCountsForksOnly) {
  // A 3 km chain whose middle edge ends at a fork: 1 junction / 3 km.
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(1000, 200),
                                   make_edge(1000, 200)};
  edges[1].junction_at_end = true;
  EXPECT_NEAR(junctions_per_km(edges), 1.0f / 3.0f, 1e-5f);
  // No forks at all -> an uninterrupted road.
  std::vector<EdgeCandidate> clean{make_edge(1000, 200), make_edge(1000, 200)};
  EXPECT_FLOAT_EQ(junctions_per_km(clean), 0.0f);
}

TEST(StretchExtractor, FinalizeFillsBucketAndInterruptionFields) {
  // An in-band run carrying per-edge histogram + interruption data; finalize
  // must aggregate all three WS-C1 fields onto the emitted stretch.
  std::vector<EdgeCandidate> edges{make_edge(1000, 200), make_edge(1000, 200),
                                   make_edge(1000, 200)};
  for (auto& e : edges) {
    e.bucket_len_m = {200.0f, 100.0f, 0.0f, 0.0f, 700.0f};
    e.density = 4;
  }
  edges[2].junction_at_end = true;
  auto s = finalize(edges);
  int sum = 0;
  for (size_t b = 0; b < kBucketCount; ++b) sum += s.bucket_shares[b];
  EXPECT_NEAR(sum, 255, 2);
  EXPECT_NEAR(s.bucket_shares[0], 170, 1); // 600 of 900 classified metres
  EXPECT_NEAR(s.bucket_shares[1], 85, 1);  // 300 of 900
  EXPECT_EQ(s.bucket_shares[2], 0);
  EXPECT_EQ(s.bucket_shares[3], 0);
  EXPECT_FLOAT_EQ(s.mean_density, 4.0f);
  EXPECT_NEAR(s.junctions_per_km, 1.0f / 3.0f, 1e-5f);
  // The scoring path is untouched by the side channel.
  EXPECT_FLOAT_EQ(s.score, s.mean_sinuosity_raw);
  EXPECT_GT(s.score, 0.0f);
}

} // namespace
