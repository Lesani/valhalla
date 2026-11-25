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
#include "loki/worker.h"
#include "midgard/encoded.h"
#include "test.h"
#include "thor/worker.h"

#include <gtest/gtest.h>

#include <set>
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

// ---- gates (patch 0030) ----
//
// B-G is a residential dead-end stub, flagged not_thru by hand as upstream's
// not_thru tests do (the enhancer does not flag it on a map this small). A gate across the column of B (heading south) is
// crossed southbound by A-D, B-G and C-F; B-G enters a dead end, so it is no
// candidate and the route crosses on A-D and flows on, where a plain point on
// the stub rides in and U-turns out. The gates carry minimum_reachability 0:
// this map has fewer nodes than the default reach of 50, below which a
// crossing only counts as a fallback.

namespace {

const std::string kStubMap = R"(
      A----B----C
      |    |    |
      |    |    |
      |    G    |
      |         |
      D----E----F
  )";

const gurka::ways kStubWays = {
    {"AB", {{"highway", "secondary"}, {"name", "AB"}}},
    {"BC", {{"highway", "secondary"}, {"name", "BC"}}},
    {"BG", {{"highway", "residential"}, {"name", "BG"}}},
    {"AD", {{"highway", "secondary"}, {"name", "AD"}}},
    {"DE", {{"highway", "secondary"}, {"name", "DE"}}},
    {"EF", {{"highway", "secondary"}, {"name", "EF"}}},
    {"CF", {{"highway", "secondary"}, {"name", "CF"}}},
};

class GateTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kStubMap, 100);
    map = gurka::buildtiles(layout, kStubWays, {}, {}, VALHALLA_BUILD_DIR "test/data/loop_gates");
    auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    const auto into_stub = std::get<0>(gurka::findEdgeByNodes(*reader, layout, "B", "G"));
    test::customize_edges(map.config, [&into_stub](const baldr::GraphId& id, baldr::DirectedEdge& e) {
      if (id == into_stub) {
        e.set_not_thru(true);
      }
    });
  }

  static std::string ll(const midgard::PointLL& p) {
    return R"("lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat());
  }

  // Halfway down the stub.
  static midgard::PointLL mid_stub() {
    const auto& b = map.nodes.at("B");
    const auto& g = map.nodes.at("G");
    return {(b.lng() + g.lng()) / 2, (b.lat() + g.lat()) / 2};
  }

  static std::string request(const std::string& middle) {
    return R"({"locations": [{)" + ll(map.nodes.at("A")) + "}, " + middle + ", {" +
           ll(map.nodes.at("C")) + R"(}], "costing": "motorcycle"})";
  }
};
gurka::map GateTest::map = {};

} // namespace

TEST_F(GateTest, PlainPointRidesIntoTheStub) {
  const auto via = "{" + ll(mid_stub()) + R"(, "type": "through"})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(via));
  // In, U-turn at the dead end, out (the through edge is split at the point).
  gurka::assert::raw::expect_path(result, {"AB", "BG", "BG", "BG", "BC"});
}

TEST_F(GateTest, GateCrossesOnAThroughRoad) {
  const auto via = "{" + ll(mid_stub()) +
                   R"(, "type": "through", "gate_heading": 180, "gate_radius": 800, "minimum_reachability": 0})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(via));
  // A-D is split where it crosses the gate and the route flows on.
  gurka::assert::raw::expect_path(result, {"AD", "AD", "DE", "EF", "CF"});
}

TEST_F(GateTest, NorthboundGateIsCrossedNorthbound) {
  // Heading north only D->A, G->B and F->C cross it: from A the route goes
  // round D-E-F and crosses northbound on F->C.
  const auto via = "{" + ll(mid_stub()) +
                   R"(, "type": "through", "gate_heading": 0, "gate_radius": 800, "minimum_reachability": 0})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(via));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "CF", "CF"});
}

TEST_F(GateTest, NarrowGateReachesThroughRoadsAtASoftCost) {
  // 150 m each side crosses only the dead-end stub; A-D and C-F cross the
  // gate's soft ends (out to 4x the radius, 500 m away), so the route takes
  // one of them rather than the stub.
  const auto via = "{" + ll(mid_stub()) +
                   R"(, "type": "through", "gate_heading": 180, "gate_radius": 150, "minimum_reachability": 0})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(via));
  gurka::assert::raw::expect_path(result, {"AD", "AD", "DE", "EF", "CF"});
}

TEST_F(GateTest, GateReachesAtMostFourfold) {
  // 60 m x 4 = 240 m still reaches only the stub: the dead-end crossing is
  // all there is, so the route takes it rather than a gate far wider than
  // asked (on a small loop that would triple its length).
  const auto via = "{" + ll(mid_stub()) +
                   R"(, "type": "through", "gate_heading": 180, "gate_radius": 60, "minimum_reachability": 0})";
  auto result = gurka::do_action(valhalla::Options::route, map, request(via));
  gurka::assert::raw::expect_path(result, {"AB", "BG", "BG", "BG", "BC"});
}

// ---- in-request reuse penalty (patch 0030) ----

TEST_F(LoopGuidanceTest, WithoutReuseTheLoopRidesBackTheSameWay) {
  const auto req = R"({"locations": [)" + loc("A") + "," +
                   loc("C") + "," + loc("A") +
                   R"(], "costing": "motorcycle"})";
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "BC", "AB"});
}

TEST_F(LoopGuidanceTest, ReuseFactorSendsTheReturnAnotherWay) {
  const auto req = R"({"locations": [)" + loc("A") + "," +
                   loc("C") + "," + loc("A") +
                   R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" +
                   R"({"reuse_factor": 4}}})";
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "FC", "EF", "DE", "AD"});
}

TEST_F(LoopGuidanceTest, ReuseClearExemptsTheStart) {
  // Every node of this map lies within 5 km of A: nothing is penalised.
  const auto req = R"({"locations": [)" + loc("A") + "," +
                   loc("C") + "," + loc("A") +
                   R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" +
                   R"({"reuse_factor": 4, "reuse_clear": 5000}}})";
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "BC", "AB"});
}

// ---- linear_cost_factors robustness (patch 0031) ----

namespace {

std::string lcf_request(const std::string& from,
                        const std::string& to,
                        const std::string& shape,
                        const std::string& extra) {
  return R"({"locations": [)" + from + "," + to +
         R"(], "costing": "motorcycle", "linear_cost_factors": [{"shape": ")" + shape +
         R"(", "factor": 8)" + extra + "}]}";
}

bool has_warning(const valhalla::Api& api, unsigned code) {
  for (const auto& w : api.info().warnings()) {
    if (w.code() == code)
      return true;
  }
  return false;
}

} // namespace

TEST_F(LoopGuidanceTest, UnwalkableLineIsSkippedNotFatal) {
  // A diagonal A->E follows no road: the edge walk fails. The route still
  // comes back (the line is skipped, warning 216) instead of error 233.
  const auto req = lcf_request(loc("A"), loc("C"), shape_of({"A", "E"}), "");
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
  EXPECT_TRUE(has_warning(result, 216));
}

TEST_F(LoopGuidanceTest, FactorPricesOnlyTheLinesDirection) {
  // A->B->C priced x8 eastbound; riding C->A westbound is not affected.
  const auto req = lcf_request(loc("C"), loc("A"), shape_of({"A", "B", "C"}), "");
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"BC", "AB"});
}

TEST_F(LoopGuidanceTest, OpposingPricesBothDirections) {
  const auto req =
      lcf_request(loc("C"), loc("A"), shape_of({"A", "B", "C"}), R"(, "opposing": true)");
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"FC", "EF", "DE", "AD"});
  EXPECT_FALSE(has_warning(result, 216));
}

// ---- seeded jitter (patch 0032) ----
//
// Two roads of equal length from A to C (over B and over D). Without jitter
// the choice is a tie; with jitter the request seed decides it: one seed
// always picks the same road, and over a handful of seeds both roads win.

namespace {

const std::string kTwinMap = R"(
          B
      A       C
          D
  )";

const gurka::ways kTwinWays = {
    {"AB", {{"highway", "secondary"}, {"name", "AB"}}},
    {"BC", {{"highway", "secondary"}, {"name", "BC"}}},
    {"AD", {{"highway", "secondary"}, {"name", "AD"}}},
    {"DC", {{"highway", "secondary"}, {"name", "DC"}}},
};

class JitterTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kTwinMap, 100);
    map = gurka::buildtiles(layout, kTwinWays, {}, {}, VALHALLA_BUILD_DIR "test/data/loop_jitter");
  }

  static std::string loc(const std::string& n) {
    const auto& p = map.nodes.at(n);
    return R"({"lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat()) + "}";
  }

  // The first road name of the A->C route for one seed.
  static std::string first_road(uint32_t seed, const std::string& cell = "0") {
    const auto req = R"({"locations": [)" + loc("A") + "," + loc("C") +
                     R"(], "costing": "motorcycle", "costing_options": {"motorcycle": )" +
                     R"({"jitter": 1.0, "jitter_cell": )" + cell + R"(, "jitter_seed": )" +
                     std::to_string(seed) + "}}}";
    auto result = gurka::do_action(valhalla::Options::route, map, req);
    const auto& leg = result.trip().routes(0).legs(0);
    return leg.node(0).edge().name(0).value();
  }
};
gurka::map JitterTest::map = {};

} // namespace

TEST_F(JitterTest, SameSeedSameRoad) {
  for (uint32_t seed : {1u, 2u, 3u}) {
    EXPECT_EQ(first_road(seed), first_road(seed)) << "seed=" << seed;
  }
}

TEST_F(JitterTest, SeedsPickBothRoads) {
  std::set<std::string> seen;
  for (uint32_t seed = 1; seed <= 12; ++seed) {
    seen.insert(first_road(seed));
  }
  EXPECT_EQ(seen.size(), 2u);
}

TEST_F(JitterTest, SpatialFieldSeedsPickBothRoads) {
  // A 150 m lattice: B and D fall in different cells.
  std::set<std::string> seen;
  for (uint32_t seed = 1; seed <= 12; ++seed) {
    seen.insert(first_road(seed, "150"));
  }
  EXPECT_EQ(seen.size(), 2u);
}

// ---- gate lookahead (patch 0033) ----
//
// A gate (heading east) between B-C and E-F. The top crossing on B-C is the
// cheaper one to reach from A, but beyond it C only offers a small loop
// (C-Q-P-C) back onto B-C westbound or a long way round over K: the next leg
// to T rides back over the arrival. With lookahead the leg drops that
// crossing and crosses on E-F, which flows on to T.

namespace {

const std::string kLookMap = R"(
          K




















      P---Q
          |
  A---B---C
  |
  D---E---F-T
  )";

const gurka::ways kLookWays = {
    {"AB", {{"highway", "secondary"}, {"name", "AB"}}},
    {"BC", {{"highway", "secondary"}, {"name", "BC"}}},
    {"CQ", {{"highway", "secondary"}, {"name", "CQ"}}},
    {"QP", {{"highway", "secondary"}, {"name", "QP"}}},
    {"PC", {{"highway", "secondary"}, {"name", "PC"}}},
    {"CK", {{"highway", "secondary"}, {"name", "CK"}}},
    {"KT", {{"highway", "secondary"}, {"name", "KT"}}},
    {"AD", {{"highway", "secondary"}, {"name", "AD"}}},
    {"DE", {{"highway", "secondary"}, {"name", "DE"}}},
    {"EF", {{"highway", "secondary"}, {"name", "EF"}}},
    {"FT", {{"highway", "secondary"}, {"name", "FT"}}},
};

class LookaheadTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kLookMap, 100);
    map = gurka::buildtiles(layout, kLookWays, {}, {}, VALHALLA_BUILD_DIR "test/data/loop_lookahead");
  }

  static std::string ll(const midgard::PointLL& p) {
    return R"("lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat());
  }

  static std::string request(int lookahead) {
    const auto& b = map.nodes.at("B");
    const auto& c = map.nodes.at("C");
    const auto& e = map.nodes.at("E");
    // Halfway between B and C, halfway down to E-F; 150 m each side.
    const midgard::PointLL gate{(b.lng() + c.lng()) / 2, (b.lat() + e.lat()) / 2};
    return R"({"locations": [{)" + ll(map.nodes.at("A")) + "}, {" + ll(gate) +
           R"(, "type": "through", "gate_heading": 90, "gate_radius": 150,)" +
           R"( "minimum_reachability": 0}, {)" + ll(map.nodes.at("T")) +
           R"(}], "costing": "motorcycle", "costing_options": {"motorcycle": {"gate_lookahead": )" +
           std::to_string(lookahead) + "}}}";
  }
};
gurka::map LookaheadTest::map = {};

} // namespace

TEST_F(LookaheadTest, WithoutLookaheadTheNextLegRidesBack) {
  auto result = gurka::do_action(valhalla::Options::route, map, request(0));
  gurka::assert::raw::expect_path(result, {"AB", "BC", "BC", "PC", "QP", "CQ", "BC", "AB", "AD",
                                           "DE", "EF", "FT"});
}

TEST_F(LookaheadTest, LookaheadCrossesWhereTheRouteFlowsOn) {
  auto result = gurka::do_action(valhalla::Options::route, map, request(2));
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "EF", "FT"});
}

TEST_F(LookaheadTest, GateWhoseEveryCrossingTurnsBackIsSkipped) {
  // A first gate reaching only B-C, from where every way on turns back, and
  // a second one across F-T: the first is dropped (warning 217) and the route
  // goes on to the second. A lone gate is never dropped (a loop needs one).
  const auto& b = map.nodes.at("B");
  const auto& c = map.nodes.at("C");
  const auto& f = map.nodes.at("F");
  const auto& t = map.nodes.at("T");
  const midgard::PointLL top{(b.lng() + c.lng()) / 2, b.lat()};
  const midgard::PointLL ft{(f.lng() + t.lng()) / 2, f.lat()};
  const auto gate = [](const midgard::PointLL& p) {
    return "{" + ll(p) + R"(, "type": "through", "gate_heading": 90, "gate_radius": 20,)" +
           R"( "minimum_reachability": 0})";
  };
  const auto req = R"({"locations": [{)" + ll(map.nodes.at("A")) + "}, " + gate(top) + ", " +
                   gate(ft) + ", {" + ll(t) +
                   R"(}], "costing": "motorcycle", "costing_options": {"motorcycle": )" +
                   R"({"gate_lookahead": 2}}})";
  auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AD", "DE", "EF", "FT", "FT"});
  bool warned = false;
  for (const auto& w : result.info().warnings())
    warned = warned || w.code() == 217;
  EXPECT_TRUE(warned);
}

// Patch 0052 (Vamoto #352, an iPhone abort when a loop was switched to the
// Adventure profile): a gate with one through crossing (B-C) and one
// destination-only crossing (E-F, a fallback candidate), both of which turn
// back. B is reached only over the residents-only A-B, so the first pass to
// the gate fails and the relaxed pass adds the fallback crossing. The
// lookahead drops B-C, and the search to the gate that is left needs the
// relaxed pass again. Before 0052 that added E-F a second time; dropping it
// after its own lookahead emptied the gate, and the next search read the
// first candidate of an empty list (absl CHECK, SIGABRT).

namespace {

const std::string kFallbackMap = R"(
    T---G
        |
        |       P---Q
        |           |
  O-A---B-----------C
    |
    D---E-----------F---H
  )";

const gurka::ways kFallbackWays = {
    {"OA", {{"highway", "secondary"}, {"name", "OA"}}},
    {"AB", {{"highway", "secondary"}, {"name", "AB"}, {"access", "destination"}}},
    {"BC", {{"highway", "secondary"}, {"name", "BC"}}},
    {"CQ", {{"highway", "secondary"}, {"name", "CQ"}}},
    {"QP", {{"highway", "secondary"}, {"name", "QP"}}},
    {"PC", {{"highway", "secondary"}, {"name", "PC"}}},
    {"BG", {{"highway", "secondary"}, {"name", "BG"}, {"oneway", "yes"}}},
    {"GT", {{"highway", "secondary"}, {"name", "GT"}}},
    {"AD", {{"highway", "secondary"}, {"name", "AD"}}},
    {"DE", {{"highway", "secondary"}, {"name", "DE"}}},
    {"EF", {{"highway", "secondary"}, {"name", "EF"}, {"access", "destination"}}},
    {"FH", {{"highway", "secondary"}, {"name", "FH"}, {"access", "destination"}}},
};

class FallbackCrossingTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(kFallbackMap, 100);
    map = gurka::buildtiles(layout, kFallbackWays, {}, {},
                            VALHALLA_BUILD_DIR "test/data/loop_fallback_crossing");
  }

  static std::string ll(const midgard::PointLL& p) {
    return R"("lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat());
  }

  static std::string request(int lookahead) {
    const auto& b = map.nodes.at("B");
    const auto& e = map.nodes.at("E");
    const auto& p = map.nodes.at("P");
    // West of P, halfway between B-C and E-F; 150 m each side.
    const midgard::PointLL gate{(b.lng() + p.lng()) / 2, (b.lat() + e.lat()) / 2};
    return R"({"locations": [{)" + ll(map.nodes.at("O")) + "}, {" + ll(gate) +
           R"(, "type": "through", "gate_heading": 90, "gate_radius": 150,)" +
           R"( "minimum_reachability": 0}, {)" + ll(map.nodes.at("T")) +
           R"(}], "costing": "motorcycle", "costing_options": {"motorcycle": {"gate_lookahead": )" +
           std::to_string(lookahead) + "}}}";
  }
};
gurka::map FallbackCrossingTest::map = {};

} // namespace

TEST_F(FallbackCrossingTest, GateWhoseFallbackCrossingTurnsBackToo) {
  // Every crossing turns back and the gate is the only one: the first leg
  // stays, as without a lookahead.
  const auto plain = gurka::do_action(valhalla::Options::route, map, request(0));
  const auto looked = gurka::do_action(valhalla::Options::route, map, request(3));
  EXPECT_EQ(gurka::detail::get_paths(plain), gurka::detail::get_paths(looked));
}

// The relaxed pass's fallback candidates (patch 0052): the same candidate is
// added once however often a location gets a relaxed pass, and another snap
// of an edge that is a candidate already stays a candidate of its own.
TEST(FilteredEdges, EachCandidateIsAddedOnce) {
  valhalla::Location loc;
  const auto add = [](google::protobuf::RepeatedPtrField<valhalla::PathEdge>* list, uint64_t id,
                      double along, valhalla::Location::SideOfStreet side) {
    auto* e = list->Add();
    e->set_graph_id(id);
    e->set_percent_along(along);
    e->set_side_of_street(side);
  };
  auto* corr = loc.mutable_correlation();
  add(corr->mutable_edges(), 1, 0.0, valhalla::Location::kNone);
  add(corr->mutable_filtered_edges(), 1, 0.25, valhalla::Location::kLeft); // same edge, other snap
  add(corr->mutable_filtered_edges(), 1, 0.0, valhalla::Location::kNone);  // the candidate itself
  add(corr->mutable_filtered_edges(), 2, 0.0, valhalla::Location::kNone);

  thor::thor_worker_t::add_filtered_edges(loc);
  thor::thor_worker_t::add_filtered_edges(loc);

  ASSERT_EQ(loc.correlation().edges_size(), 3);
  EXPECT_EQ(loc.correlation().edges(1).graph_id(), 1u);
  EXPECT_DOUBLE_EQ(loc.correlation().edges(1).percent_along(), 0.25);
  EXPECT_EQ(loc.correlation().edges(2).graph_id(), 2u);
  EXPECT_EQ(loc.correlation().filtered_edges_size(), 3);
}

TEST_F(FallbackCrossingTest, AnotherSnapOfACandidateEdgeIsStillAFallback) {
  // The origin's only first-pass candidate is A-D at node A; its interior
  // snap on the same edge is filtered by the preferred side. An excluded
  // point lies between the two, so only the interior snap can ride on to
  // D-E. The relaxed pass must add it although its edge is a candidate.
  const std::string req =
      R"({"locations": [{"lat": -0.004042400287459576, "lon": 0.0022457779374775422,)"
      R"( "preferred_side": "opposite", "radius": 100, "search_cutoff": 100,)"
      R"( "minimum_reachability": 0}, {"lat": -0.0053898670499461015,)"
      R"( "lon": 0.0035932446999640674, "search_cutoff": 2, "minimum_reachability": 0}],)"
      R"( "exclude_locations": [{"lat": -0.0037729069349622707, "lon": 0.0017966223499820337}],)"
      R"( "costing": "motorcycle",)"
      R"( "date_time": {"type": 1, "value": "2026-11-25T12:00"}})";
  const auto result = gurka::do_action(valhalla::Options::route, map, req);
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(FallbackCrossingTest, LocationWithoutACandidateEdgeIsNoPath) {
  // Whatever empties a location's candidates, the search answers 442 (no
  // path); before 0052 bidirectional A* read the first candidate unchecked.
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  loki::loki_worker_t loki(map.config, reader);
  thor::thor_worker_t thor(map.config, reader);
  const std::string req = R"({"locations": [{)" + ll(map.nodes.at("O")) + "}, {" +
                          ll(map.nodes.at("T")) + R"(}], "costing": "motorcycle"})";
  valhalla::Api api;
  ParseApi(req, valhalla::Options::route, api);
  loki.route(api);
  auto* destination = api.mutable_options()->mutable_locations(1)->mutable_correlation();
  destination->mutable_edges()->Clear();
  destination->mutable_filtered_edges()->Clear();
  try {
    thor.route(api);
    FAIL() << "a location without a candidate edge was routed";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 442u); }
}
