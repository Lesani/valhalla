// Patch 0037 (#230): plain motorcycle_curvy must not leave and rejoin a
// primary via a turn channel, while a route starting on that link remains
// routable. The exact route assertions run only in the integrated engine gate.
#include "gurka.h"
#include "test.h"
#include "midgard/encoded.h"
#include <rapidjson/document.h>
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
void expect_exact_shape(const std::vector<midgard::PointLL>& points,
                        const gurka::map& tiles = map) {
  const auto result = gurka::do_action(Options::trace_attributes, tiles, exact_trace_request(points));
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

TEST_F(TurnChannelHops, ExactWalkViaRetainsPartialDestinationSpan) {
  const auto end = map.nodes.at("C").PointAlongSegment(map.nodes.at("D"), 0.5);
  const std::string finish = R"({"lon":)" + std::to_string(end.lng()) +
                             R"(,"lat":)" + std::to_string(end.lat()) + "}";
  const auto route = gurka::do_action(Options::route, map, request(loc("A"), finish));
  auto points = midgard::decode<std::vector<midgard::PointLL>>(route.trip().routes(0).legs(0).shape());
  ASSERT_GE(points.size(), 4);
  points.insert(points.begin() + 1, points[0].PointAlongSegment(points[1], 0.5));
  expect_exact_shape(points);
}


namespace {
gurka::map bent_edge_map;
const midgard::PointLL bent_a(13, 47), bent_b(13.001, 47), bent_c(13.001, 47.001);
class BentEdgeExactWalk : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    bent_edge_map = gurka::buildtiles(
        {{"A", bent_a}, {"B", bent_b}, {"C", bent_c}},
        {{"ABC", {{"highway", "primary"}, {"name", "Bent road"}}}}, {}, {},
        VALHALLA_BUILD_DIR "test/data/bent_edge_exact_walk");
  }
};
} // namespace

TEST_F(BentEdgeExactWalk, PartialOriginAtInteriorBendBothDirections) {
  // B is geometry inside one edge, not a graph junction. Routing proves that
  // these partial paths exist before asking exact matching to preserve them.
  for (const auto& finish : {bent_c, bent_a}) {
    const auto location = [](const auto& p) {
      return R"({"lon":)" + std::to_string(p.lng()) +
             R"(,"lat":)" + std::to_string(p.lat()) + "}";
    };
    const auto route = gurka::do_action(Options::route, bent_edge_map,
                                      request(location(bent_b), location(finish)));
    const auto& leg = route.trip().routes(0).legs(0);
    ASSERT_EQ(leg.node_size(), 2); // exactly one graph edge
    const auto routed = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
    // Reverse clipping retains B twice. Preserve that actual input too, as
    // well as the minimal two-point partial-origin counterexample.
    const std::vector<midgard::PointLL> expected = finish == bent_a
        ? std::vector<midgard::PointLL>{bent_b, bent_b, bent_a}
        : std::vector<midgard::PointLL>{bent_b, bent_c};
    ASSERT_EQ(routed, expected);
    expect_exact_shape({bent_b, finish}, bent_edge_map);
    expect_exact_shape(routed, bent_edge_map);
  }
}

TEST_F(BentEdgeExactWalk, PartialOriginAndPartialDestination) {
  expect_exact_shape({bent_b, bent_b.PointAlongSegment(bent_c, 0.5)}, bent_edge_map);
}

TEST_F(BentEdgeExactWalk, IncomingPartialOriginKeepsInteriorGraphVertex) {
  expect_exact_shape({bent_a.PointAlongSegment(bent_b, 0.5), bent_b, bent_c}, bent_edge_map);
}

TEST_F(BentEdgeExactWalk, PartialOriginRejectsOffEdgePoint) {
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, bent_edge_map,
      exact_trace_request({bent_b, midgard::PointLL(13.0011, 47.0005), bent_c})));
}

TEST_F(BentEdgeExactWalk, PartialOriginRejectsBacktracking) {
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, bent_edge_map,
      exact_trace_request({bent_b, bent_b.PointAlongSegment(bent_c, 0.75),
                          bent_b.PointAlongSegment(bent_c, 0.25), bent_c})));
}

TEST_F(BentEdgeExactWalk, IncomingPartialOriginCannotSkipBend) {
  EXPECT_ANY_THROW(gurka::do_action(Options::trace_attributes, bent_edge_map,
      exact_trace_request({bent_a.PointAlongSegment(bent_b, 0.5), bent_c})));
}
