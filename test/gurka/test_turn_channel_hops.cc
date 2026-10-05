// Patch 0037 (#230): plain motorcycle_curvy must not leave and rejoin a
// primary via a turn channel, while a route starting on that link remains
// routable. The exact route assertions run only in the integrated engine gate.
#include "gurka.h"
#include "test.h"
#include "midgard/encoded.h"
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

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

namespace {
std::vector<midgard::PointLL> mainline_shape() {
  const auto route = gurka::do_action(Options::route, map, request(loc("A"), loc("D")));
  return midgard::decode<std::vector<midgard::PointLL>>(route.trip().routes(0).legs(0).shape());
}
std::string exact_trace_request(const std::vector<midgard::PointLL>& points) {
  rapidjson::Document doc;
  doc.Parse(R"({"costing":"motorcycle_curvy","shape_match":"edge_walk","filters":{"action":"include","attributes":["shape","edge.begin_shape_index","edge.end_shape_index","edge.names","edge.use","edge.length","edge.road_class"]}})");
  auto encoded = midgard::encode(points);
  doc.AddMember("encoded_polyline", rapidjson::Value(encoded.c_str(), doc.GetAllocator()),
                doc.GetAllocator());
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  return buffer.GetString();
}
void expect_exact_shape(const std::vector<midgard::PointLL>& points) {
  const auto result = gurka::do_action(Options::trace_attributes, map, exact_trace_request(points));
  const auto& leg = result.trip().routes(0).legs(0);
  const auto expected = midgard::decode<std::vector<midgard::PointLL>>(midgard::encode(points));
  EXPECT_EQ(midgard::decode<std::vector<midgard::PointLL>>(leg.shape()), expected);
  uint32_t previous = 0;
  for (const auto& node : leg.node()) {
    if (!node.has_edge()) continue;
    EXPECT_EQ(node.edge().begin_shape_index(), previous);
    EXPECT_LE(node.edge().begin_shape_index(), node.edge().end_shape_index());
    previous = node.edge().end_shape_index();
  }
  EXPECT_EQ(previous, expected.size() - 1);
}
} // namespace

TEST_F(TurnChannelHops, ExactWalkPreservesOriginalAndOnEdgeViaVertices) {
  auto points = mainline_shape();
  ASSERT_GE(points.size(), 3);
  expect_exact_shape(points);
  const auto midpoint = points[0].PointAlongSegment(points[1], 0.5);
  points.insert(points.begin() + 1, midpoint);
  expect_exact_shape(points);
}

TEST_F(TurnChannelHops, ExactWalkRejectsOffEdgeVertex) {
  auto points = mainline_shape();
  auto midpoint = points[0].PointAlongSegment(points[1], 0.5);
  midpoint = midgard::PointLL(midpoint.lng(), midpoint.lat() + 0.0001);
  points.insert(points.begin() + 1, midpoint);
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, map, exact_trace_request(points)));
}

TEST_F(TurnChannelHops, ExactWalkRejectsBacktrackingOnAnEdge) {
  auto points = mainline_shape();
  const auto a = points[0].PointAlongSegment(points[1], 0.75);
  const auto b = points[0].PointAlongSegment(points[1], 0.25);
  points.insert(points.begin() + 1, {a, b});
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, map, exact_trace_request(points)));
}

TEST_F(TurnChannelHops, ExactWalkRejectsMissingGraphJunction) {
  auto points = mainline_shape();
  ASSERT_GE(points.size(), 4);
  points.erase(points.begin() + 1);
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, map, exact_trace_request(points)));
}
