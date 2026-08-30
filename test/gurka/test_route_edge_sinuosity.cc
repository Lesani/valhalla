#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/constants.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace valhalla;

// better_mc_routing patch 0020: the valhalla-format /route serializer emits a
// per-leg `edges` array carrying the fork's 0-255 sinuosity byte alongside
// enough bookkeeping for a client to length-weight it into a "twisty %"
// figure. The array is unconditional -- no request knob, no attribute filter
// -- so these tests do plain `route` calls.

namespace {

constexpr double kGridSizeMeters = 100.;

// A tight zigzag (A..M, 100 m legs -> ~100 m corner radius, well inside the
// curve-density metric's 175 m bin) and a dead-straight run (N..R). Each is a
// single way with no junctions, so each route is one directed edge per leg.
const std::string ascii_map = R"(
   B D F H J L
  A C E G I K M


  N---O---P---Q
  )";

const gurka::ways ways = {
    {"ABCDEFGHIJKLM", {{"highway", "secondary"}}},
    {"NOPQ", {{"highway", "secondary"}}},
};

struct edge_record {
  uint32_t sinuosity;
  double length;
  uint32_t begin_shape_index;
  uint32_t end_shape_index;
};

std::vector<edge_record> read_edges(const rapidjson::Value& leg) {
  std::vector<edge_record> out;
  EXPECT_TRUE(leg.HasMember("edges")) << "leg carries no per-edge array";
  EXPECT_TRUE(leg["edges"].IsArray());
  for (const auto& e : leg["edges"].GetArray()) {
    EXPECT_TRUE(e.HasMember("sinuosity") && e["sinuosity"].IsUint());
    EXPECT_TRUE(e.HasMember("length") && e["length"].IsNumber());
    EXPECT_TRUE(e.HasMember("begin_shape_index") && e["begin_shape_index"].IsUint());
    EXPECT_TRUE(e.HasMember("end_shape_index") && e["end_shape_index"].IsUint());
    // exactly the four keys the clients read -- keep the payload compact
    EXPECT_EQ(e.MemberCount(), 4u);
    out.push_back({e["sinuosity"].GetUint(), e["length"].GetDouble(),
                   e["begin_shape_index"].GetUint(), e["end_shape_index"].GetUint()});
  }
  return out;
}

gurka::map& test_map() {
  static gurka::map m = []() {
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSizeMeters);
    return gurka::buildtiles(layout, ways, {}, {}, "test/data/route_edge_sinuosity");
  }();
  return m;
}

TEST(RouteEdgeSinuosity, EdgesArrayIsEmittedForEveryLeg) {
  auto result = gurka::do_action(valhalla::Options::route, test_map(), {"A", "M"}, "motorcycle");
  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_json);
  ASSERT_FALSE(json.HasParseError());

  ASSERT_TRUE(json["trip"]["legs"].IsArray());
  ASSERT_EQ(json["trip"]["legs"].GetArray().Size(), 1u);
  const auto& leg = json["trip"]["legs"][0];

  const auto edges = read_edges(leg);
  ASSERT_FALSE(edges.empty());

  // The shape indices tile the leg's shape end to end.
  const auto shape = midgard::decode<std::vector<midgard::PointLL>>(leg["shape"].GetString());
  const auto last_index = static_cast<uint32_t>(shape.size() - 1);
  EXPECT_EQ(edges.front().begin_shape_index, 0u);
  EXPECT_EQ(edges.back().end_shape_index, last_index);
  for (size_t i = 0; i < edges.size(); ++i) {
    EXPECT_LT(edges[i].begin_shape_index, edges[i].end_shape_index);
    EXPECT_LE(edges[i].end_shape_index, last_index);
    if (i + 1 < edges.size()) {
      EXPECT_EQ(edges[i].end_shape_index, edges[i + 1].begin_shape_index);
    }
    EXPECT_LE(edges[i].sinuosity, 255u);
  }

  // Lengths are in the leg summary's units, so they sum to it.
  double sum = 0.;
  for (const auto& e : edges) {
    EXPECT_GT(e.length, 0.);
    sum += e.length;
  }
  EXPECT_NEAR(sum, leg["summary"]["length"].GetDouble(), 0.01);
}

TEST(RouteEdgeSinuosity, ByteTracksTheGeometry) {
  auto zigzag = gurka::do_action(valhalla::Options::route, test_map(), {"A", "M"}, "motorcycle");
  auto zigzag_json = gurka::convert_to_json(zigzag, Options::Format::Options_Format_json);
  auto straight = gurka::do_action(valhalla::Options::route, test_map(), {"N", "Q"}, "motorcycle");
  auto straight_json = gurka::convert_to_json(straight, Options::Format::Options_Format_json);

  const auto zigzag_edges = read_edges(zigzag_json["trip"]["legs"][0]);
  const auto straight_edges = read_edges(straight_json["trip"]["legs"][0]);
  ASSERT_FALSE(zigzag_edges.empty());
  ASSERT_FALSE(straight_edges.empty());

  uint32_t zigzag_max = 0, straight_max = 0;
  for (const auto& e : zigzag_edges)
    zigzag_max = std::max(zigzag_max, e.sinuosity);
  for (const auto& e : straight_edges)
    straight_max = std::max(straight_max, e.sinuosity);

  // A straight line is not sinuous; the zigzag is, and the byte says so.
  EXPECT_EQ(straight_max, 0u);
  EXPECT_GT(zigzag_max, 0u);
}

TEST(RouteEdgeSinuosity, LengthFollowsRequestUnits) {
  auto km = gurka::do_action(valhalla::Options::route, test_map(), {"A", "M"}, "motorcycle",
                             {{"/units", "kilometers"}});
  auto km_json = gurka::convert_to_json(km, Options::Format::Options_Format_json);
  auto mi = gurka::do_action(valhalla::Options::route, test_map(), {"A", "M"}, "motorcycle",
                             {{"/units", "miles"}});
  auto mi_json = gurka::convert_to_json(mi, Options::Format::Options_Format_json);

  const auto km_edges = read_edges(km_json["trip"]["legs"][0]);
  const auto mi_edges = read_edges(mi_json["trip"]["legs"][0]);
  ASSERT_EQ(km_edges.size(), mi_edges.size());
  ASSERT_FALSE(km_edges.empty());

  double km_sum = 0., mi_sum = 0.;
  for (size_t i = 0; i < km_edges.size(); ++i) {
    EXPECT_EQ(km_edges[i].sinuosity, mi_edges[i].sinuosity);
    km_sum += km_edges[i].length;
    mi_sum += mi_edges[i].length;
  }
  EXPECT_NEAR(mi_sum, km_sum * midgard::kMilePerKm, 0.01);
  EXPECT_NEAR(mi_sum, mi_json["trip"]["legs"][0]["summary"]["length"].GetDouble(), 0.01);
}

} // namespace
