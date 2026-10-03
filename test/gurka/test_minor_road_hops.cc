// Gurka integration tests for minor-road hops (vamoto patch 0036, #209 phase 7).
//
// The owner's ride 2026-10-03: a loop left the B159 at Pfarrwerfen and the
// B164 at Muehlbach am Hochkoenig for a few hundred metres of side street and
// rejoined the same road. On these maps a straight main road A..D has a side
// street next to it that leaves and rejoins it. A plain route stays on the
// main road; before 0036 the loop layers (nice-road weight, jitter) and a
// gate whose line reaches the side street first made some requests take it.
//
// These use the `motorcycle` costing: motorcycle_curvy inherits the loop
// layers and the hop guard through MotorcycleCost (the curvy costing crashes
// under gurka's bidirectional A*, see patch 0019).

#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

using namespace valhalla;

namespace {

// 10 m grid: the main road B-C is 120 m, the side street B-F-G-C 128 m.
const std::string kTwinMap = R"(
      A----B-----------C----D
            F---------G
  )";

gurka::ways twin_ways(const std::string& side_highway) {
  return {
      {"AB", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "AB"}}},
      {"BC", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "BC"}}},
      {"CD", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "CD"}}},
      {"BFGC", {{"highway", side_highway}, {"maxspeed", "50"}, {"name", "Side"}}},
  };
}

std::string loc(const gurka::map& map, const std::string& n) {
  const auto& p = map.nodes.at(n);
  return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
}

std::string request(const gurka::map& map,
                    const std::string& from,
                    const std::string& to,
                    const std::string& options) {
  return R"({"locations": [)" + from + "," + to +
         R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" + options + "}}";
}

std::vector<std::string> names(const valhalla::Api& result) {
  std::vector<std::string> out;
  for (const auto& leg : result.trip().routes(0).legs()) {
    for (const auto& node : leg.node()) {
      if (node.has_edge() && node.edge().name_size() > 0) {
        out.push_back(node.edge().name(0).value());
      }
    }
  }
  return out;
}

bool rides_side(const valhalla::Api& result) {
  const auto n = names(result);
  return std::find(n.begin(), n.end(), "Side") != n.end();
}

// A loop request's layers: per-road jitter (every road its own draw) and the
// nice-road weight.
std::string loop_options(uint32_t seed) {
  return R"({"jitter": 1.0, "jitter_cell": 0, "jitter_seed": )" + std::to_string(seed) +
         R"(, "nice_weight": 2.0})";
}

class ResidentialTwinTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kTwinMap, 10);
    map = gurka::buildtiles(layout, twin_ways("residential"), {}, {},
                            VALHALLA_BUILD_DIR "test/data/minor_hops_residential");
  }
};
gurka::map ResidentialTwinTest::map = {};

class LivingStreetTwinTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kTwinMap, 10);
    map = gurka::buildtiles(layout, twin_ways("living_street"), {}, {},
                            VALHALLA_BUILD_DIR "test/data/minor_hops_living");
  }
};
gurka::map LivingStreetTwinTest::map = {};

class UnclassifiedTwinTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kTwinMap, 10);
    map = gurka::buildtiles(layout, twin_ways("unclassified"), {}, {},
                            VALHALLA_BUILD_DIR "test/data/minor_hops_unclassified");
  }
};
gurka::map UnclassifiedTwinTest::map = {};

} // namespace

TEST_F(ResidentialTwinTest, APlainRouteStaysOnTheMainRoad) {
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 request(map, loc(map, "A"), loc(map, "D"), "{}"));
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
}

TEST_F(ResidentialTwinTest, NoSeedSendsALoopThroughTheSideStreet) {
  // A residential way earns no nice-road discount and no favourable jitter
  // draw and pays the minor-road factor: whatever the main road draws, the
  // side street costs more.
  for (uint32_t seed = 1; seed <= 24; ++seed) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   request(map, loc(map, "A"), loc(map, "D"), loop_options(seed)));
    EXPECT_FALSE(rides_side(result)) << "seed=" << seed;
  }
}

TEST_F(ResidentialTwinTest, ALoopStartingInTheSideStreetStillRidesIt) {
  // A town start: the route begins on the residential street and reaches
  // the main road through it.
  const auto& f = map.nodes.at("F");
  const auto& g = map.nodes.at("G");
  const std::string mid = R"({"lon": )" + std::to_string((f.lng() + g.lng()) / 2) +
                          R"(, "lat": )" + std::to_string((f.lat() + g.lat()) / 2) + "}";
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 request(map, mid, loc(map, "D"), loop_options(1)));
  const auto n = names(result);
  ASSERT_FALSE(n.empty());
  EXPECT_EQ(n.front(), "Side");
  EXPECT_EQ(n.back(), "CD");
}

TEST_F(LivingStreetTwinTest, NoSeedSendsALoopThroughTheLivingStreet) {
  for (uint32_t seed = 1; seed <= 24; ++seed) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   request(map, loc(map, "A"), loc(map, "D"), loop_options(seed)));
    EXPECT_FALSE(rides_side(result)) << "seed=" << seed;
  }
}

TEST_F(UnclassifiedTwinTest, TheHopGuardKeepsALoopOnTheMainRoad) {
  // An unclassified road keeps its jitter draw and its curve, but leaving
  // the primary for it and coming back pays the hop guard twice.
  for (uint32_t seed = 1; seed <= 24; ++seed) {
    auto result = gurka::do_action(valhalla::Options::route, map,
                                   request(map, loc(map, "A"), loc(map, "D"), loop_options(seed)));
    EXPECT_FALSE(rides_side(result)) << "seed=" << seed;
  }
}

TEST_F(UnclassifiedTwinTest, APlainRouteIsUnchanged) {
  auto result = gurka::do_action(valhalla::Options::route, map,
                                 request(map, loc(map, "A"), loc(map, "D"), "{}"));
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
}

// ---- a gate next to a side street ----
//
// 20 m grid. The gate travels almost south (170 deg), so its line runs almost
// east-west and meets the side street E-F (60 m south of the main road) about
// 340 m west of where it meets the main road. A leg ends on whichever crossing
// it reaches cheapest and the next leg continues from there: the side street
// crossing comes first, so the route left the main road for it and came back.
// Within 400 m of a main-road crossing a side-street crossing is a fallback.

namespace {

const std::string kGateMap = R"(
      A----B---------C--------------D
           |         |
           |         |
           E---------F
  )";

const gurka::ways kGateWays = {
    {"AB", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "AB"}}},
    {"BC", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "BC"}}},
    {"CD", {{"highway", "primary"}, {"maxspeed", "50"}, {"name", "CD"}}},
    {"BEFC", {{"highway", "residential"}, {"maxspeed", "50"}, {"name", "Side"}}},
};

class GateSideStreetTest : public ::testing::Test {
protected:
  static gurka::map map;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kGateMap, 20);
    map = gurka::buildtiles(layout, kGateWays, {}, {}, VALHALLA_BUILD_DIR "test/data/minor_hops_gate");
  }
};
gurka::map GateSideStreetTest::map = {};

} // namespace

TEST_F(GateSideStreetTest, TheGateIsCrossedOnTheMainRoad) {
  // The gate centre: on the side street, 20 m east of E.
  const auto& e = map.nodes.at("E");
  const auto& f = map.nodes.at("F");
  const double lng = e.lng() + (f.lng() - e.lng()) * 0.1;
  const std::string gate = R"({"lon": )" + std::to_string(lng) + R"(, "lat": )" +
                           std::to_string(e.lat()) +
                           R"(, "type": "through", "gate_heading": 170, "gate_radius": 600,)"
                           R"( "minimum_reachability": 0})";
  const std::string req = R"({"locations": [)" + loc(map, "A") + "," + gate + "," +
                          loc(map, "D") + R"(], "costing": "motorcycle"})";
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  EXPECT_FALSE(rides_side(result));
}
