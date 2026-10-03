// Gurka integration tests for the loop-guidance layers (vamoto patches 0029+).
//
// The map has a SHORT direct road (A-B-C) and a LONGER detour (A-D-E-F-C).
// Without guidance the router takes the short road; a corridor drawn along the
// detour makes the detour the cheaper way, while a corridor along the direct
// road changes nothing.
//
// These use the `motorcycle` costing: motorcycle_curvy inherits the guidance
// factor through MotorcycleCost::EdgeCost (proven in the CostInline tests;
// the curvy costing crashes under gurka's bidirectional A*, see patch 0019).

#include "gurka.h"
#include "midgard/encoded.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace valhalla;

namespace {

const std::string kAsciiMap = R"(
      A----B----C
      |         |
      |         |
      |         |
      D----E----F
  )";

const gurka::ways kWays = {
    {"AB", {{"highway", "secondary"}, {"name", "AB"}}},
    {"BC", {{"highway", "secondary"}, {"name", "BC"}}},
    {"AD", {{"highway", "secondary"}, {"name", "AD"}}},
    {"DE", {{"highway", "secondary"}, {"name", "DE"}}},
    {"EF", {{"highway", "secondary"}, {"name", "EF"}}},
    {"FC", {{"highway", "secondary"}, {"name", "FC"}}},
};

std::string json_escape(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '\\')
      out += '\\';
    out += c;
  }
  return out;
}

class LoopGuidanceTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
    map = gurka::buildtiles(layout, kWays, {}, {}, VALHALLA_BUILD_DIR "test/data/loop_guidance");
  }

  static std::string shape_of(const std::vector<std::string>& nodes) {
    std::vector<midgard::PointLL> pts;
    for (const auto& n : nodes)
      pts.push_back(map.nodes.at(n));
    return json_escape(midgard::encode(pts));
  }

  static std::string loc(const std::string& n) {
    const auto& p = map.nodes.at(n);
    return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
  }

  // /route A->C with a costing_options body for motorcycle.
  static std::string request(const std::string& options) {
    return R"({"locations": [)" + loc("A") + "," + loc("C") +
           R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" + options + "}}";
  }
};
gurka::map LoopGuidanceTest::map = {};

} // namespace

TEST_F(LoopGuidanceTest, NoGuidanceTakesTheShortRoad) {
  auto result = gurka::do_action(valhalla::Options::route, map, request("{}"));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(LoopGuidanceTest, CorridorPullsOntoTheDetour) {
  // A narrow band along D-E-F with a steep slope: the direct road is ~400 m
  // outside it and pays the cap, the detour pays about x1.
  const auto opts = R"({"corridor": ")" + shape_of({"D", "E", "F"}) +
                    R"(", "corridor_width": 60, "corridor_slope": 4, "corridor_max": 5})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(opts));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FC"});
}

TEST_F(LoopGuidanceTest, CorridorOnTheShortRoadChangesNothing) {
  const auto opts = R"({"corridor": ")" + shape_of({"A", "B", "C"}) +
                    R"(", "corridor_width": 60, "corridor_slope": 4, "corridor_max": 5})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(opts));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(LoopGuidanceTest, SoftCorridorKeepsTheShortRoad) {
  // A low cap (x1.5) on the 2-cell direct road (cost 3) does not beat the
  // 6-cell detour: the corridor is a guide, not a constraint.
  const auto opts = R"({"corridor": ")" + shape_of({"D", "E", "F"}) +
                    R"(", "corridor_width": 60, "corridor_slope": 4, "corridor_max": 1.5})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(opts));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
