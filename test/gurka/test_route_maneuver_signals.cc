#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/constants.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace valhalla;

// better_mc_routing patch 0021: the valhalla-format /route serializer emits
// `traffic_signal` in two places -- once per maneuver (a light at the
// maneuver's own junction) and once per entry of patch 0020's per-leg
// `edges` array (a light at that edge's END). Both are emitted ONLY when
// true, so "absent" means false and the payload stays compact.
//
// NOT covered here: the bounded walk-back over `internal_intersection`
// connectors in the maneuver hunk. `internal_intersection` is assigned by
// graphenhancer's IsIntersectionInternal heuristic, which a hand-drawn
// gurka grid cannot reliably trigger; asserting on it would test the
// heuristic rather than our code. That path is covered by inspection plus
// a live-tile smoke check against the Europe tiles.
//
// Costing is `auto`: motorcycle_curvy crashes under gurka's bidirectional
// A*, and patch 0020's test uses `motorcycle` for the same reason.

namespace {

constexpr double kGridSizeMeters = 100.;

// Two plus-shaped junctions. C carries a node-tagged traffic light; G is
// identical but unsignalised, which is what pins "emitted only when true".
const std::string ascii_map = R"(
      B         F
      |         |
   A--C--D   E--G--H
      |         |
      I         J
  )";

const gurka::ways ways = {
    {"AC", {{"highway", "primary"}}},   {"CD", {{"highway", "primary"}}},
    {"BCI", {{"highway", "secondary"}}}, {"EG", {{"highway", "primary"}}},
    {"GH", {{"highway", "primary"}}},   {"FGJ", {{"highway", "secondary"}}},
};

const gurka::nodes nodes = {
    {"C", {{"highway", "traffic_signals"}}},
};

gurka::map& test_map() {
  static gurka::map m = []() {
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, kGridSizeMeters);
    return gurka::buildtiles(layout, ways, nodes, {}, "test/data/route_maneuver_signals");
  }();
  return m;
}

rapidjson::Document route_json(const std::string& from, const std::string& to) {
  auto result = gurka::do_action(valhalla::Options::route, test_map(), {from, to}, "auto");
  auto json = gurka::convert_to_json(result, Options::Format::Options_Format_json);
  EXPECT_FALSE(json.HasParseError());
  return json;
}

// Count the maneuvers of leg 0 that carry `"traffic_signal": true`.
size_t signalled_maneuvers(const rapidjson::Value& leg) {
  EXPECT_TRUE(leg.HasMember("maneuvers"));
  size_t n = 0;
  for (const auto& m : leg["maneuvers"].GetArray()) {
    if (m.HasMember("traffic_signal")) {
      EXPECT_TRUE(m["traffic_signal"].IsBool());
      EXPECT_TRUE(m["traffic_signal"].GetBool()) << "the key is emitted only when true";
      ++n;
    }
  }
  return n;
}

// Index of the shape vertex closest to `p`, and its distance in metres.
std::pair<uint32_t, double> closest_vertex(const std::vector<midgard::PointLL>& shape,
                                           const midgard::PointLL& p) {
  uint32_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < shape.size(); ++i) {
    const double d = shape[i].Distance(p);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return {best, best_d};
}

} // namespace

TEST(RouteManeuverSignals, TurnAtASignalisedNodeCarriesTheFlag) {
  auto json = route_json("A", "I");
  ASSERT_TRUE(json["trip"]["legs"].IsArray());
  const auto& leg = json["trip"]["legs"][0];

  // Exactly one junction on this route, and it is lit.
  EXPECT_EQ(signalled_maneuvers(leg), 1u);

  // It is the turn, not the depart and not the arrive.
  const auto maneuvers = leg["maneuvers"].GetArray();
  ASSERT_GE(maneuvers.Size(), 3u);
  EXPECT_TRUE(maneuvers[1].HasMember("traffic_signal"));
}

TEST(RouteManeuverSignals, TurnAtAnUnsignalisedNodeOmitsTheKey) {
  auto json = route_json("E", "J");
  const auto& leg = json["trip"]["legs"][0];
  EXPECT_EQ(signalled_maneuvers(leg), 0u);
  for (const auto& m : leg["maneuvers"].GetArray()) {
    EXPECT_FALSE(m.HasMember("traffic_signal"));
  }
}

TEST(RouteManeuverSignals, DepartManeuverNeverCarriesTheFlag) {
  auto json = route_json("A", "I");
  const auto& leg = json["trip"]["legs"][0];
  ASSERT_TRUE(leg["maneuvers"].IsArray());
  ASSERT_GE(leg["maneuvers"].GetArray().Size(), 1u);
  EXPECT_FALSE(leg["maneuvers"][0].HasMember("traffic_signal"))
      << "node 0 is the origin: there is no previous edge, the union degenerates";
}

TEST(RouteManeuverSignals, PerEdgeArrayFlagsTheEdgeEndingAtTheSignal) {
  auto json = route_json("A", "D");
  const auto& leg = json["trip"]["legs"][0];
  ASSERT_TRUE(leg.HasMember("edges") && leg["edges"].IsArray());

  size_t flagged = 0;
  uint32_t flagged_end = 0;
  for (const auto& e : leg["edges"].GetArray()) {
    if (!e.HasMember("traffic_signal")) {
      continue;
    }
    ASSERT_TRUE(e["traffic_signal"].IsBool());
    EXPECT_TRUE(e["traffic_signal"].GetBool());
    ++flagged;
    flagged_end = e["end_shape_index"].GetUint();
  }
  ASSERT_EQ(flagged, 1u) << "only the edge that ends at C is lit";

  // The flagged edge ends at C: patch 0021 anchors a per-edge signal at the
  // edge's END, so `end_shape_index` must land on C's vertex.
  const auto shape = midgard::decode<std::vector<midgard::PointLL>>(leg["shape"].GetString());
  const auto c = closest_vertex(shape, test_map().nodes.at("C"));
  EXPECT_LT(c.second, 10.) << "C must be on the route shape";
  EXPECT_EQ(flagged_end, c.first);
}

TEST(RouteManeuverSignals, EdgesWithoutASignalOmitTheKey) {
  auto json = route_json("E", "H");
  const auto& leg = json["trip"]["legs"][0];
  ASSERT_TRUE(leg.HasMember("edges") && leg["edges"].IsArray());
  ASSERT_GT(leg["edges"].GetArray().Size(), 0u);
  for (const auto& e : leg["edges"].GetArray()) {
    EXPECT_FALSE(e.HasMember("traffic_signal"));
  }
}

TEST(RouteManeuverSignals, SinuosityKeysAreUnchanged) {
  auto json = route_json("A", "D");
  const auto& leg = json["trip"]["legs"][0];
  ASSERT_TRUE(leg.HasMember("edges") && leg["edges"].IsArray());
  ASSERT_GT(leg["edges"].GetArray().Size(), 0u);
  for (const auto& e : leg["edges"].GetArray()) {
    EXPECT_TRUE(e.HasMember("sinuosity") && e["sinuosity"].IsUint());
    EXPECT_TRUE(e.HasMember("length") && e["length"].IsNumber());
    EXPECT_TRUE(e.HasMember("begin_shape_index") && e["begin_shape_index"].IsUint());
    EXPECT_TRUE(e.HasMember("end_shape_index") && e["end_shape_index"].IsUint());
    // patch 0021 is purely additive: 4 keys, plus the optional flag.
    EXPECT_GE(e.MemberCount(), 4u);
    EXPECT_LE(e.MemberCount(), 5u);
  }
}
