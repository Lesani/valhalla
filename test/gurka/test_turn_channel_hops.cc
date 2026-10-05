// Patch 0037 (#230): plain motorcycle_curvy must not leave and rejoin a
// primary via a turn channel, while a route starting on that link remains
// routable. The exact route assertions run only in the integrated engine gate.
#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::string kMap = R"(
  A----B-----------C----D
        E-----------F
)";
const gurka::ways kWays = {
    {"AB", {{"highway", "primary"}, {"name", "B178"}}},
    {"BC", {{"highway", "primary"}, {"name", "B178"}}},
    {"CD", {{"highway", "primary"}, {"name", "B178"}}},
    {"BEFC", {{"highway", "primary_link"}, {"turn_channel", "true"}, {"internal_intersection", "true"}, {"name", "Link"}}},
};

gurka::map map;
std::string loc(const std::string& node) {
  const auto& p = map.nodes.at(node);
  return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
}
std::string request(const std::string& from, const std::string& to) {
  return R"({"locations":[)" + from + "," + to + R"(],"costing":"motorcycle_curvy"})";
}

class TurnChannelHops : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    map = gurka::buildtiles(gurka::detail::map_to_coordinates(kMap, 10), kWays, {}, {},
                            VALHALLA_BUILD_DIR "test/data/turn_channel_hops");
  }
};
} // namespace

TEST_F(TurnChannelHops, PlainCurvyStaysOnTheMainlineInBothDirections) {
  gurka::assert::raw::expect_path(gurka::do_action(Options::route, map, request(loc("A"), loc("D"))),
                                  {"B178", "B178", "B178"});
  gurka::assert::raw::expect_path(gurka::do_action(Options::route, map, request(loc("D"), loc("A"))),
                                  {"B178", "B178", "B178"});
}

TEST_F(TurnChannelHops, AStartOnTheLinkCanStillReachTheMainline) {
  auto result = gurka::do_action(Options::route, map, request(loc("E"), loc("D")));
  ASSERT_FALSE(result.trip().routes(0).legs().empty());
}

namespace {
const std::string kMotorwayAccessMap = R"(
  A----B
       |
       C----D
)";
const gurka::ways kMotorwayAccessWays = {
    {"AB", {{"highway", "motorway"}, {"oneway", "yes"}, {"name", "A1"}}},
    {"BC", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"name", "A1 exit"}}},
    {"CD", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Access"}}},
};
gurka::map motorway_access_map;

class MotorwayLinkAccess : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    motorway_access_map = gurka::buildtiles(
        gurka::detail::map_to_coordinates(kMotorwayAccessMap, 10), kMotorwayAccessWays, {}, {},
        VALHALLA_BUILD_DIR "test/data/turn_channel_motorway_access");
  }
};
} // namespace

TEST_F(MotorwayLinkAccess, CurvyCanUseARequiredMotorwayRampAccess) {
  const auto& a = motorway_access_map.nodes.at("A");
  const auto& d = motorway_access_map.nodes.at("D");
  const auto point = [](const auto& p) {
    return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
  };
  const std::string req = R"({"locations":[)" + point(a) + "," + point(d) +
                          R"(],"costing":"motorcycle_curvy"})";
  auto result = gurka::do_action(Options::route, motorway_access_map, req);
  ASSERT_FALSE(result.trip().routes(0).legs().empty());
}
