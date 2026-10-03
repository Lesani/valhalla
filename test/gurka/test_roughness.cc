// Gurka integration tests for the road-roughness tolerance (vamoto patch 0035).
//
// The map has a SHORT direct road (A-B-C) and a LONGER paved detour
// (A-D-E-F-C), all tagged 60 km/h (a rough surface rides at 50: Valhalla's
// speed assigner). The direct road's surface is varied per fixture. A rider whose
// tolerance covers that surface keeps the short road; a stricter rider pays
// the roughness multiplier on it and takes the detour.
//
// These use the `motorcycle` costing: motorcycle_curvy inherits the factor
// through MotorcycleCost::EdgeCost (the curvy costing crashes under gurka's
// bidirectional A*, see patch 0019).

#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

#include <string>

using namespace valhalla;

namespace {

const std::string kAsciiMap = R"(
      A----B----C
      |         |
      |         |
      |         |
      D----E----F
  )";

gurka::ways ways_with_direct_surface(const std::string& surface) {
  return {
      {"AB", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "AB"}, {"surface", surface}}},
      {"BC", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "BC"}, {"surface", surface}}},
      {"AD", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "AD"}}},
      {"DE", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "DE"}}},
      {"EF", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "EF"}}},
      {"FC", {{"highway", "secondary"}, {"maxspeed", "60"}, {"name", "FC"}}},
  };
}

std::string loc(const gurka::map& map, const std::string& n) {
  const auto& p = map.nodes.at(n);
  return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
}

std::string request(const gurka::map& map, const std::string& options) {
  return R"({"locations": [)" + loc(map, "A") + "," + loc(map, "C") +
         R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" + options + "}}";
}

// Cobblestones: baldr kPavedRough (2), free of the stock trail surface factor.
class CobbleTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
    map = gurka::buildtiles(layout, ways_with_direct_surface("cobblestone"), {}, {},
                            VALHALLA_BUILD_DIR "test/data/roughness_cobble");
  }
};
gurka::map CobbleTest::map = {};

// A compacted road: kCompacted (3), ridden by a trail-friendly rider.
class CompactedTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
    map = gurka::buildtiles(layout, ways_with_direct_surface("compacted"), {}, {},
                            VALHALLA_BUILD_DIR "test/data/roughness_compacted");
  }
};
gurka::map CompactedTest::map = {};

} // namespace

TEST_F(CobbleTest, WithoutAToleranceTheShortRoadWins) {
  auto result = gurka::do_action(valhalla::Options::route, map, request(map, "{}"));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(CobbleTest, ATolerantRiderKeepsTheShortRoad) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, request(map, R"({"max_roughness": 2})"));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(CobbleTest, AStrictRiderTakesThePavedDetour) {
  // Two steps above smooth asphalt: 5x on the 2-cell road beats the 6-cell
  // detour.
  auto result =
      gurka::do_action(valhalla::Options::route, map, request(map, R"({"max_roughness": 0})"));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FC"});
}

TEST_F(CobbleTest, AFloatToleranceIsRounded) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, request(map, R"({"max_roughness": 0.2})"));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FC"});
}

TEST_F(CompactedTest, AnAdventureToleranceKeepsTheShortRoad) {
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 request(map, R"({"use_trails": 1.0, "max_roughness": 6})"));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(CompactedTest, AStreetToleranceAvoidsItEvenWhenTrailsAreWelcome) {
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 request(map, R"({"use_trails": 1.0, "max_roughness": 1})"));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FC"});
}
