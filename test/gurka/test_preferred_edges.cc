// Gurka integration tests for the preferred-trail edge bias (patch 0019).
//
// The map has a SHORT direct path (A-B-C) parallel to a LONGER detour
// (A-D-E-F-C). With no bias the router takes the short path. Registering the
// detour's directed edges as the preferred set and turning the strength up
// pushes the router onto the longer trail; a mild strength leaves it on the
// short road (a soft bias, never a hard include).
//
// These use the `motorcycle` costing. motorcycle_curvy inherits the preferred
// factor through MotorcycleCost::EdgeCost (base.cost) -- proven directly in the
// CostInline unit test MotorcycleCurvyCost.PreferredEdgesInherited, because the
// curvy costing has a pre-existing crash under gurka's bidirectional A* on
// these ASCII tiles (reproduces with an EMPTY preferred set, i.e. unrelated to
// patch 0019; motorcycle_curvy + preferred_edges routes fine via
// valhalla_service).

#include "baldr/graphreader.h"
#include "gurka.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace valhalla;

namespace {

// A B C along the top row; D E F along the bottom row. The top path (AB, BC) is
// 2 cells; the detour (AD, DE, EF, FC) is 4 cells, so it is strictly longer.
const std::string kAsciiMap = R"(
      A----B----C
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

std::string join_ids(const std::vector<uint64_t>& ids) {
  std::string out;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i)
      out += ",";
    out += std::to_string(ids[i]);
  }
  return out;
}

class PreferredEdgesTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kAsciiMap, 100);
    map = gurka::buildtiles(layout, kWays, {}, {}, VALHALLA_BUILD_DIR "test/data/preferred_edges");
  }

  // The forward directed-edge GraphIds for a set of node->node hops.
  static std::vector<uint64_t>
  edge_ids(const std::vector<std::pair<std::string, std::string>>& hops) {
    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    std::vector<uint64_t> ids;
    for (const auto& hop : hops) {
      auto e = gurka::findEdgeByNodes(reader, map.nodes, hop.first, hop.second);
      ids.push_back(std::get<0>(e).value);
    }
    return ids;
  }

  // The A->C detour edges (AD, DE, EF, FC), in traversal direction.
  static std::vector<uint64_t> detour_ids() {
    return edge_ids({{"A", "D"}, {"D", "E"}, {"E", "F"}, {"F", "C"}});
  }

  // Compose a /route A->C request with a preferred set in costing_options.
  static std::string request(const std::string& costing,
                             const std::vector<uint64_t>& ids,
                             float factor) {
    const auto& a = map.nodes.at("A");
    const auto& c = map.nodes.at("C");
    std::string opts;
    if (!ids.empty() || factor != 1.0f) {
      opts = R"(, "costing_options": {")" + costing + R"(": {"preferred_edges": [)" + join_ids(ids) +
             R"(], "preferred_factor": )" + std::to_string(factor) + "}}";
    }
    return R"({"locations": [{"lon": )" + std::to_string(a.lng()) + R"(, "lat": )" +
           std::to_string(a.lat()) + R"(}, {"lon": )" + std::to_string(c.lng()) + R"(, "lat": )" +
           std::to_string(c.lat()) + R"(}], "costing": ")" + costing + R"(")" + opts + "}";
  }
};
gurka::map PreferredEdgesTest::map = {};

} // namespace

TEST_F(PreferredEdgesTest, MotorcyclePrefersSetWhenFactorHigh) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, request("motorcycle", detour_ids(), 8.0f));
  // F=8 makes the 2-cell direct path cost 16 cells; the 4-cell detour is
  // cheaper, so the router rides the preferred trail.
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FC"});
}

TEST_F(PreferredEdgesTest, EmptySetIsNoOp) {
  auto result = gurka::do_action(valhalla::Options::route, map, request("motorcycle", {}, 1.0f));
  // No preferred set -> the plain shortest path (the direct road).
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(PreferredEdgesTest, FactorOneIsNoOp) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, request("motorcycle", detour_ids(), 1.0f));
  // Strength Off (F=1.0): the set is present but the bias is a no-op.
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(PreferredEdgesTest, MildFactorKeepsShortRoadWhenDetourTooLong) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, request("motorcycle", detour_ids(), 1.5f));
  // F=1.5 makes the direct path cost 3 cells vs the 4-cell detour, so the soft
  // bias is not enough to leave the short road.
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}

TEST_F(PreferredEdgesTest, NonMemberRouteStillSucceeds) {
  // A preferred set that contains NONE of the edges on any A->C path (the
  // reverse detour edges). With every A->C edge a non-member and F=8 the whole
  // graph scales uniformly, so the route still succeeds on the short path --
  // no starvation of reachability.
  auto ids = edge_ids({{"D", "A"}, {"E", "D"}});
  auto result = gurka::do_action(valhalla::Options::route, map, request("motorcycle", ids, 8.0f));
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
