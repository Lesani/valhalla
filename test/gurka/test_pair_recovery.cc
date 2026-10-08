// Gurka integration tests for pair recovery (vamoto patch 0050).
//
// A route request's errors name the pair (or location) they belong to, so a
// client can recover that interval alone: 447 (label budget exhausted) and 442
// (no path) carry the failed pair's original location indices, 171 (no edges
// near a location) that location's. A location's prune_hierarchy flag makes the
// one search ending there keep the config's default hierarchy limits although
// the costing disables pruning; a capped request's pair longer than
// max_distance_disable_hierarchy_culling gets that flag from loki (warning 205),
// and the route response lists the pruned pairs. Between pairs and while a
// trip is assembled the tile cache drops what it holds once it is over its size.

#include "gurka.h"
#include "loki/worker.h"
#include "sif/costfactory.h"
#include "sif/hierarchylimits.h"
#include "test.h"
#include "thor/worker.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using namespace valhalla;

namespace {

std::string ll(const midgard::PointLL& p) {
  return R"("lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat());
}

std::string loc(const midgard::PointLL& p, const std::string& extra = "") {
  return "{" + ll(p) + extra + "}";
}

const std::string kPrune = R"(, "prune_hierarchy": true)";
const std::string kThrough = R"(, "type": "through")";

std::string gate(const midgard::PointLL& p, int heading, int radius, const std::string& extra = "") {
  return loc(p, R"(, "type": "through", "gate_heading": )" + std::to_string(heading) +
                    R"(, "gate_radius": )" + std::to_string(radius) +
                    R"(, "minimum_reachability": 0)" + extra);
}

// A motorcycle /route request; `costing` is the motorcycle costing options
// object's body, `extra` more top-level members.
std::string request(const std::vector<std::string>& locations,
                    const std::string& costing = "",
                    const std::string& extra = "") {
  std::string locs;
  for (const auto& l : locations) {
    locs += (locs.empty() ? "" : ", ") + l;
  }
  return R"({"locations": [)" + locs + R"(], "costing": "motorcycle", "costing_options": )" +
         R"({"motorcycle": {)" + costing + "}}" + extra + "}";
}

std::string cap(uint32_t c) {
  return R"("search_label_cap": )" + std::to_string(c);
}

const std::string kDepartAt = R"(, "date_time": {"type": 1, "value": "2025-08-15T14:50"})";
const std::string kArriveBy = R"(, "date_time": {"type": 2, "value": "2025-08-15T14:50"})";

// The error a request fails with, nullopt when it routes.
std::optional<valhalla_exception_t>
route_error(const gurka::map& map,
            const std::string& req,
            const std::shared_ptr<baldr::GraphReader>& reader = {}) {
  try {
    gurka::do_action(Options::route, map, req, reader);
    return std::nullopt;
  } catch (const valhalla_exception_t& e) { return e; }
}

// What a service caller gets back for an error.
std::string error_body(const valhalla_exception_t& e) {
  Api api;
  return serialize_error(e, api);
}

bool warned(const Api& api, unsigned code) {
  return std::any_of(api.info().warnings().begin(), api.info().warnings().end(),
                     [code](const auto& w) { return w.code() == code; });
}

std::vector<std::vector<std::string>> legs_of(const Api& result) {
  std::vector<std::vector<std::string>> legs;
  for (const auto& leg : result.trip().routes(0).legs()) {
    std::vector<std::string> names;
    for (const auto& node : leg.node()) {
      if (node.has_edge()) {
        names.push_back(node.edge().name(0).value());
      }
    }
    legs.push_back(names);
  }
  return legs;
}

// ---------------------------------------------------------------------------
// Workers driven directly, with a probe into each path search.

class ProbeThor : public thor::thor_worker_t {
public:
  using thor::thor_worker_t::thor_worker_t;
  void track(const thor::PathAlgorithm::expansion_callback_t& callback) {
    bidir_astar.set_track_expansion(callback);
    timedep_forward.set_track_expansion(callback);
    timedep_reverse.set_track_expansion(callback);
  }
  uint32_t searches() const {
    return path_searches_;
  }
  // The hierarchy limits on the costing the current search runs with.
  const std::vector<HierarchyLimits>& limits() {
    return mode_costing[static_cast<uint32_t>(mode)]->GetHierarchyLimits();
  }
  bool default_limits() {
    return mode_costing[static_cast<uint32_t>(mode)]->DefaultHierarchyLimits();
  }
};

// What a search saw when it began: the costing's level-2 limits and its
// default-limits flag.
struct SearchLimits {
  uint32_t max_up_transitions;
  float expand_within_dist;
  bool default_limits;
};

// loki + thor + the valhalla serializer on one map and reader.
struct Engine {
  std::shared_ptr<baldr::GraphReader> reader;
  loki::loki_worker_t loki;
  ProbeThor thor;
  // by search number (1 = the request's first path search)
  std::map<uint32_t, SearchLimits> seen;
  uint32_t seen_search = 0;

  Engine(const gurka::map& map, std::shared_ptr<baldr::GraphReader> r = {})
      : reader(r ? r : test::make_clean_graphreader(map.config.get_child("mjolnir"))),
        loki(map.config, reader), thor(map.config, reader) {
  }

  // Records the limits each search of the next requests begins with.
  void record_limits() {
    thor.track([this](baldr::GraphReader&, const baldr::GraphId, const baldr::GraphId, const char*,
                      const Expansion_EdgeStatus, float, uint32_t, float,
                      const Expansion_ExpansionType, const uint8_t, const TravelMode) {
      if (thor.searches() == seen_search) {
        return;
      }
      seen_search = thor.searches();
      const auto& level2 = thor.limits()[2];
      seen[seen_search] = {level2.max_up_transitions(), level2.expand_within_dist(),
                           thor.default_limits()};
    });
  }

  // Routes `req` through loki and thor into `api`; throws the request's error.
  void route(const std::string& req, Api& api) {
    api.Clear();
    seen.clear();
    seen_search = 0;
    ParseApi(req, Options::route, api);
    loki.route(api);
    thor.route(api);
  }
};

} // namespace

// ---------------------------------------------------------------------------
// Exceptions.

TEST(PairRecoveryErrors, IndicesSurviveTheTableLookupCopyAndSerialization) {
  const valhalla_exception_t plain{447};
  EXPECT_EQ(plain.location_index, -1);
  EXPECT_EQ(plain.destination_index, -1);
  const valhalla_exception_t extra{442, "some detail"};
  EXPECT_EQ(extra.location_index, -1);
  EXPECT_EQ(extra.destination_index, -1);

  const valhalla_exception_t pair{447, 3, 4};
  EXPECT_EQ(pair.code, 447u);
  EXPECT_EQ(pair.http_code, 400u);
  EXPECT_EQ(pair.message, plain.message);
  EXPECT_EQ(pair.location_index, 3);
  EXPECT_EQ(pair.destination_index, 4);

  EXPECT_FALSE(pair.pruned);
  const valhalla_exception_t pruned{442, 1, 2, true};
  EXPECT_TRUE(pruned.pruned);
  EXPECT_TRUE(valhalla_exception_t{pruned}.pruned);
  EXPECT_FALSE(valhalla_exception_t{442}.pruned);
  EXPECT_NE(error_body(pruned).find(R"("pruned":true)"), std::string::npos);
  EXPECT_EQ(error_body(pair).find("pruned"), std::string::npos);

  const valhalla_exception_t copied = pair;
  EXPECT_EQ(copied.location_index, 3);
  EXPECT_EQ(copied.destination_index, 4);
  valhalla_exception_t assigned{171};
  assigned = pair;
  EXPECT_EQ(assigned.location_index, 3);
  EXPECT_EQ(assigned.destination_index, 4);
  // A table entry built after a pair error carries no index of its own.
  EXPECT_EQ(valhalla_exception_t{447}.location_index, -1);

  const auto body = error_body(pair);
  EXPECT_NE(body.find(R"("error_code":447)"), std::string::npos) << body;
  EXPECT_NE(body.find(R"("location_index":3)"), std::string::npos) << body;
  EXPECT_NE(body.find(R"("destination_index":4)"), std::string::npos) << body;
  const auto none = error_body(plain);
  EXPECT_EQ(none.find("location_index"), std::string::npos) << none;
  EXPECT_EQ(none.find("destination_index"), std::string::npos) << none;
  const auto location_only = error_body(valhalla_exception_t{171, 2, -1});
  EXPECT_NE(location_only.find(R"("location_index":2)"), std::string::npos) << location_only;
  EXPECT_EQ(location_only.find("destination_index"), std::string::npos) << location_only;
}

// ---------------------------------------------------------------------------
// A 6x6 motorcycle grid at 100 m; the top row and right column are primary.

namespace {

const std::vector<std::string> kRows = {"ABCDEF", "GHIJKL", "MNOPQR", "STUVWX", "YZ0123", "456789"};

std::string grid_ascii() {
  std::string out = "\n";
  for (size_t r = 0; r < kRows.size(); ++r) {
    std::string row, bars;
    for (size_t c = 0; c < kRows[r].size(); ++c) {
      row += (c ? "-" : "") + std::string(1, kRows[r][c]);
      bars += (c ? " " : "") + std::string("|");
    }
    out += row + "\n";
    if (r + 1 < kRows.size()) {
      out += bars + "\n";
    }
  }
  // an island road, reachable from nowhere
  out += "\n\n\n\n                     a-b\n";
  return out;
}

gurka::ways grid_ways() {
  gurka::ways ways;
  const size_t last = kRows.front().size() - 1;
  for (size_t r = 0; r < kRows.size(); ++r) {
    for (size_t c = 0; c < kRows[r].size(); ++c) {
      if (c < last) {
        const std::string name = {kRows[r][c], kRows[r][c + 1]};
        ways[name] = {{"highway", r == 0 ? "primary" : "residential"}, {"name", name}};
      }
      if (r + 1 < kRows.size()) {
        const std::string name = {kRows[r][c], kRows[r + 1][c]};
        ways[name] = {{"highway", c == last ? "primary" : "residential"}, {"name", name}};
      }
    }
  }
  ways["ab"] = {{"highway", "residential"}, {"name", "ab"}};
  return ways;
}

class PairErrorTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(grid_ascii(), 100);
    map = gurka::buildtiles(layout, grid_ways(), {}, {},
                            VALHALLA_BUILD_DIR "test/data/pair_recovery_grid");
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
  static void TearDownTestSuite() {
    reader.reset();
  }

  static std::string at(const std::string& node, const std::string& extra = "") {
    return loc(map.nodes.at(node), extra);
  }

  // The smallest cap the pair from -> to routes with on its own.
  static uint32_t
  smallest_cap(const std::string& from, const std::string& to, const std::string& extra) {
    for (uint32_t c = 1; c <= 20000; ++c) {
      if (!route_error(map, request({at(from), at(to)}, cap(c), extra), reader)) {
        return c;
      }
    }
    return 0;
  }

  // A cap the short pairs A-B and 8-9 route with and the long pair B-8 does not.
  static uint32_t only_long_pair_exhausts(const std::string& extra) {
    const uint32_t shorts = std::max(smallest_cap("A", "B", extra), smallest_cap("8", "9", extra));
    const uint32_t long_pair = smallest_cap("B", "8", extra);
    EXPECT_GT(shorts, 0u);
    EXPECT_GT(long_pair, shorts) << "the long pair must need more labels";
    return shorts;
  }
};
gurka::map PairErrorTest::map = {};
std::shared_ptr<baldr::GraphReader> PairErrorTest::reader = {};

} // namespace

namespace {

// A costing that fails while loki correlates the locations: its edge filter
// (the candidate search's Allowed) throws `Failure`.
template <typename Failure> class FailingFilterCost final : public sif::DynamicCost {
public:
  explicit FailingFilterCost(const Costing& options)
      : DynamicCost(options, sif::TravelMode::kDrive, baldr::kMotorcycleAccess) {
  }
  bool Allowed(const baldr::DirectedEdge*,
               const bool,
               const sif::EdgeLabel&,
               const baldr::graph_tile_ptr&,
               const baldr::GraphId&,
               const uint64_t,
               const uint32_t,
               uint8_t&,
               uint8_t&) const override {
    return true;
  }
  bool AllowedReverse(const baldr::DirectedEdge*,
                      const sif::EdgeLabel&,
                      const baldr::DirectedEdge*,
                      const baldr::graph_tile_ptr&,
                      const baldr::GraphId&,
                      const uint64_t,
                      const uint32_t,
                      uint8_t&,
                      uint8_t&) const override {
    return true;
  }
  sif::Cost EdgeCost(const baldr::DirectedEdge*,
                     const baldr::TransitDeparture*,
                     const uint32_t) const override {
    return {};
  }
  sif::Cost EdgeCost(const baldr::DirectedEdge*,
                     const baldr::GraphId&,
                     const baldr::graph_tile_ptr&,
                     const baldr::TimeInfo&,
                     uint8_t&) const override {
    return {};
  }
  sif::Cost TransitionCost(const baldr::DirectedEdge*,
                           const baldr::NodeInfo*,
                           const sif::EdgeLabel&,
                           const baldr::graph_tile_ptr&,
                           const std::function<baldr::LimitedGraphReader()>&) const override {
    return {};
  }
  sif::Cost TransitionCostReverse(const uint32_t,
                                  const baldr::NodeInfo*,
                                  const baldr::DirectedEdge*,
                                  const baldr::DirectedEdge*,
                                  const baldr::graph_tile_ptr&,
                                  const baldr::GraphId&,
                                  const std::function<baldr::LimitedGraphReader()>&,
                                  const bool,
                                  const sif::InternalTurn) const override {
    return {};
  }
  float AStarCostFactor() const override {
    return 0.1f;
  }
  bool Allowed(const baldr::DirectedEdge*, const baldr::graph_tile_ptr&, uint16_t) const override {
    if constexpr (std::is_same_v<Failure, std::bad_alloc>) {
      throw std::bad_alloc();
    } else {
      throw Failure("candidate search failed");
    }
  }
};

class ProbeLoki : public loki::loki_worker_t {
public:
  using loki::loki_worker_t::loki_worker_t;
  template <typename Failure> void correlation_fails_with() {
    factory.Register(Costing::motorcycle, [](const Costing& options) -> sif::cost_ptr_t {
      return std::make_shared<FailingFilterCost<Failure>>(options);
    });
  }
};

} // namespace

TEST_F(PairErrorTest, OutOfMemoryWhileCorrelatingIs448Not171) {
  ProbeLoki loki(map.config, reader);
  loki.correlation_fails_with<std::bad_alloc>();
  Api api;
  ParseApi(request({at("A"), at("9")}), Options::route, api);
  try {
    loki.route(api);
    FAIL() << "correlation must run out of memory";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 448u);
    EXPECT_EQ(e.http_code, 503u);
    const auto body = error_body(e);
    EXPECT_NE(body.find(R"("error_code":448)"), std::string::npos) << body;
    EXPECT_NE(body.find(R"("status_code":503)"), std::string::npos) << body;
  }
  // any other failure there stays 171
  ProbeLoki other(map.config, reader);
  other.correlation_fails_with<std::runtime_error>();
  api.Clear();
  ParseApi(request({at("A"), at("9")}), Options::route, api);
  try {
    other.route(api);
    FAIL() << "correlation must fail";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 171u); }
}

TEST_F(PairErrorTest, OnlyPairTwoExhaustsDepartAt) {
  for (const std::string& extra : {std::string(), kDepartAt}) {
    const uint32_t c = only_long_pair_exhausts(extra);
    const auto req = request({at("A"), at("B"), at("8"), at("9")}, cap(c), extra);
    const auto e = route_error(map, req, reader);
    ASSERT_TRUE(e) << "the long pair must exhaust";
    EXPECT_EQ(e->code, 447u);
    EXPECT_EQ(e->location_index, 1);
    EXPECT_EQ(e->destination_index, 2);
    const auto body = error_body(*e);
    EXPECT_NE(body.find(R"("location_index":1)"), std::string::npos) << body;
    EXPECT_NE(body.find(R"("destination_index":2)"), std::string::npos) << body;
  }
}

TEST_F(PairErrorTest, OnlyPairTwoExhaustsArriveBy) {
  // arrive_by routes the pairs last to first; the error still names the pair
  // by its origin and destination.
  const uint32_t c = only_long_pair_exhausts(kArriveBy);
  const auto req = request({at("A"), at("B"), at("8"), at("9")}, cap(c), kArriveBy);
  const auto e = route_error(map, req, reader);
  ASSERT_TRUE(e) << "the long pair must exhaust";
  EXPECT_EQ(e->code, 447u);
  EXPECT_EQ(e->location_index, 1);
  EXPECT_EQ(e->destination_index, 2);
  EXPECT_NE(error_body(*e).find(R"("location_index":1)"), std::string::npos);
}

TEST_F(PairErrorTest, LookaheadExhaustionNamesTheGateAndTheNextLocation) {
  // A gate across B-C right after the start, the destination far across the
  // grid: the `ahead` search from the gate's crossing runs from a copy of the
  // gate, so its exhaustion names the pair (gate, next).
  const auto& b = map.nodes.at("B");
  const auto& c = map.nodes.at("C");
  const midgard::PointLL mid{(b.lng() + c.lng()) / 2, b.lat()};
  const auto req = [&](uint32_t c) {
    return request({at("A"), gate(mid, 90, 20), at("9")}, R"("gate_lookahead": 2, )" + cap(c));
  };
  Engine engine(map, reader);
  Api api;
  bool ahead_exhausted = false;
  for (uint32_t c = 1; c <= 5000 && !ahead_exhausted; ++c) {
    try {
      engine.route(req(c), api);
      break;
    } catch (const valhalla_exception_t& e) {
      ASSERT_EQ(e.code, 447u) << "cap " << c;
      // leg to the gate, then the lookahead from it
      if (engine.thor.searches() == 2) {
        EXPECT_EQ(e.location_index, 1) << "cap " << c;
        EXPECT_EQ(e.destination_index, 2) << "cap " << c;
        ahead_exhausted = true;
      } else {
        EXPECT_EQ(e.location_index, 0) << "cap " << c;
        EXPECT_EQ(e.destination_index, 1) << "cap " << c;
      }
    }
  }
  EXPECT_TRUE(ahead_exhausted) << "no cap let the first leg route and stopped the lookahead";
}

TEST_F(PairErrorTest, NoPathCarriesPairOrigin) {
  const auto island = at("a", R"(, "minimum_reachability": 0)");
  for (const std::string& extra : {std::string(), kDepartAt, kArriveBy}) {
    const auto e = route_error(map, request({at("A"), at("9"), island}, "", extra), reader);
    ASSERT_TRUE(e);
    EXPECT_EQ(e->code, 442u) << extra;
    EXPECT_EQ(e->location_index, 1) << extra;
    EXPECT_EQ(e->destination_index, 2) << extra;
  }
}

TEST_F(PairErrorTest, NoEdgesNearCarriesLocation) {
  // 1 km below the grid, searched within 50 m only
  const auto& g = map.nodes.at("4");
  const midgard::PointLL nowhere{g.lng(), g.lat() - 0.009};
  const auto e =
      route_error(map,
                  request({at("A"), at("9"), loc(nowhere, R"(, "search_cutoff": 50, "radius": 0)"),
                           at("B")}),
                  reader);
  ASSERT_TRUE(e);
  EXPECT_EQ(e->code, 171u);
  EXPECT_EQ(e->location_index, 2);
  EXPECT_EQ(e->destination_index, -1);
  const auto body = error_body(*e);
  EXPECT_NE(body.find(R"("location_index":2)"), std::string::npos) << body;
  EXPECT_EQ(body.find("destination_index"), std::string::npos) << body;
}

// ---------------------------------------------------------------------------
// A gate skipped by the lookahead (patch 0033): the pairs after it keep their
// original indices. S starts west of a small residential grid; the first gate
// crosses B-C, from where every way on turns back; the second crosses F-T.

namespace {

const std::string kSkipMap = R"(
                          P---Q
                              |
  S---s---r---q---A---B-------C
  |   |   |   |   |
  m---n---o---p---D---E---F-------T

                                       y-z
  )";

gurka::ways skip_ways() {
  gurka::ways ways;
  for (const std::string name : {"AB", "BC", "CQ", "QP", "PC", "AD", "DE", "EF", "FT"}) {
    ways[name] = {{"highway", "secondary"}, {"name", name}};
  }
  for (const std::string name :
       {"Ss", "sr", "rq", "qA", "Sm", "sn", "ro", "qp", "mn", "no", "op", "pD", "yz"}) {
    ways[name] = {{"highway", "residential"}, {"name", name}};
  }
  return ways;
}

} // namespace

TEST(PairRecoveryGateSkip, GateSkipKeepsOriginalIndex) {
  const auto layout = gurka::detail::map_to_coordinates(kSkipMap, 100);
  const auto map = gurka::buildtiles(layout, skip_ways(), {}, {},
                                     VALHALLA_BUILD_DIR "test/data/pair_recovery_skip");
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto& b = map.nodes.at("B");
  const auto& c = map.nodes.at("C");
  const auto& f = map.nodes.at("F");
  const auto& t = map.nodes.at("T");
  const midgard::PointLL top{(b.lng() + c.lng()) / 2, b.lat()};
  const midgard::PointLL ft{(f.lng() + t.lng()) / 2, f.lat()};
  const auto req = [&](const std::string& last, const std::string& costing) {
    return request({loc(map.nodes.at("S")), gate(top, 90, 20), gate(ft, 90, 20), last},
                   R"("gate_lookahead": 2)" + costing);
  };
  const auto to_t = loc(t);

  {
    // The first gate is skipped (warning 217) and the route goes on.
    auto result = gurka::do_action(Options::route, map, req(to_t, ""), reader);
    EXPECT_TRUE(warned(result, 217));
  }

  // After the skip the last pair cannot route: 442 names it by its original
  // indices (2, 3), not its position after the skip (1, 2).
  {
    const auto island = loc(map.nodes.at("y"), R"(, "minimum_reachability": 0)");
    const auto e = route_error(map, req(island, ""), reader);
    ASSERT_TRUE(e);
    EXPECT_EQ(e->code, 442u);
    EXPECT_EQ(e->location_index, 2);
    EXPECT_EQ(e->destination_index, 3);
  }

  // Every exhausted search names a pair of the request: (0, 1) the first leg,
  // (1, 2) the lookahead from the first gate, (0, 2) the leg over the skipped
  // gate, (2, 3) the last leg or its lookahead. The leg over the skipped gate
  // exhausts at some cap: recovery then acts on the interval 0-2.
  std::set<std::pair<int64_t, int64_t>> seen;
  for (uint32_t c = 1; c <= 5000; ++c) {
    const auto e = route_error(map, req(to_t, ", " + cap(c)), reader);
    if (!e) {
      break;
    }
    ASSERT_EQ(e->code, 447u) << "cap " << c;
    seen.insert({e->location_index, e->destination_index});
  }
  const std::set<std::pair<int64_t, int64_t>> legit = {{0, 1}, {1, 2}, {0, 2}, {2, 3}};
  for (const auto& p : seen) {
    EXPECT_TRUE(legit.count(p)) << "pair " << p.first << "-" << p.second;
  }
  EXPECT_TRUE(seen.count({0, 2})) << "no cap exhausted the leg over the skipped gate";
}

// ---------------------------------------------------------------------------
// Hierarchy pruning per location. A to F: the primary road B-C-D-E is a long
// detour, the residential B-G-H-E a short cut. The config prunes level 2
// (residential) after one upward transition beyond 150 m, so a pruned search
// from A rides the primary; with disable_hierarchy_pruning the short cut wins.
// x-y is an island road running south, 100 m below E-F.

namespace {

std::string prune_ascii() {
  std::string out = "\n    C-----D\n";
  for (int i = 0; i < 30; ++i) {
    out += "    |     |\n";
  }
  out += "A---B     E---F\n    |     | x\n    G-----H\n            y\n";
  return out;
}

const gurka::ways kPruneWays = {
    {"AB", {{"highway", "residential"}, {"name", "AB"}}},
    {"BC", {{"highway", "primary"}, {"name", "BC"}}},
    {"CD", {{"highway", "primary"}, {"name", "CD"}}},
    {"DE", {{"highway", "primary"}, {"name", "DE"}}},
    {"EF", {{"highway", "residential"}, {"name", "EF"}}},
    {"BG", {{"highway", "residential"}, {"name", "BG"}}},
    {"GH", {{"highway", "residential"}, {"name", "GH"}}},
    {"HE", {{"highway", "residential"}, {"name", "HE"}}},
    {"xy", {{"highway", "residential"}, {"name", "xy"}}},
};

const std::vector<std::string> kShortCut = {"AB", "BG", "GH", "HE", "EF"};
const std::vector<std::string> kPrimaryRoad = {"AB", "BC", "CD", "DE", "EF"};

std::vector<std::string> reversed(std::vector<std::string> names) {
  std::reverse(names.begin(), names.end());
  return names;
}

const std::string kNoPruning = R"("disable_hierarchy_pruning": true)";

class PruneTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(prune_ascii(), 100);
    map = gurka::buildtiles(layout, kPruneWays, {}, {},
                            VALHALLA_BUILD_DIR "test/data/pair_recovery_prune", config());
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
  static void TearDownTestSuite() {
    reader.reset();
  }

  static std::unordered_map<std::string, std::string> config() {
    std::unordered_map<std::string, std::string> c;
    for (const std::string alg : {"bidirectional_astar", "unidirectional_astar"}) {
      c["thor." + alg + ".hierarchy_limits.max_up_transitions.2"] = "0";
      c["thor." + alg + ".hierarchy_limits.expand_within_distance.2"] = "150";
    }
    // no culling: the tests set prune_hierarchy themselves
    c["service_limits.max_distance_disable_hierarchy_culling"] = "10000000";
    c["service_limits.hierarchy_limits.allow_modification"] = "true";
    return c;
  }

  static std::string at(const std::string& node, const std::string& extra = "") {
    return loc(map.nodes.at(node), extra);
  }

  static Api route(const std::string& req, std::string* json = nullptr) {
    return gurka::do_action(Options::route, map, req, reader, json);
  }
};
gurka::map PruneTest::map = {};
std::shared_ptr<baldr::GraphReader> PruneTest::reader = {};

} // namespace

TEST_F(PruneTest, PrunedFlagOnOnePairOnly) {
  for (const std::string& extra : {std::string(), kDepartAt, kArriveBy}) {
    std::string json;
    const auto result =
        route(request({at("A"), at("F"), at("A", kPrune), at("F")}, kNoPruning, extra), &json);
    const auto legs = legs_of(result);
    ASSERT_EQ(legs.size(), 3u);
    EXPECT_EQ(legs[0], kShortCut) << extra;
    EXPECT_EQ(legs[1], reversed(kPrimaryRoad)) << extra;
    EXPECT_EQ(legs[2], kShortCut) << extra;
    EXPECT_NE(json.find(R"("pruned_pairs":[[1,2]])"), std::string::npos) << json;
  }
}

TEST_F(PruneTest, PrunedFlagNoopWhenCostingPrunes) {
  std::string flagged_json, plain_json;
  const auto flagged = route(request({at("A"), at("F", kPrune)}), &flagged_json);
  const auto plain = route(request({at("A"), at("F")}), &plain_json);
  EXPECT_EQ(legs_of(flagged), legs_of(plain));
  EXPECT_EQ(legs_of(plain).front(), kPrimaryRoad);
  EXPECT_EQ(flagged_json.find("pruned_pairs"), std::string::npos) << flagged_json;
  EXPECT_EQ(plain_json.find("pruned_pairs"), std::string::npos) << plain_json;
}

TEST_F(PruneTest, NoPathOnAPrunedPairSaysPruned) {
  // The island x-y has no road to A: a 442 either way. Only the search the
  // caller's flag pruned says so; with a pruning costing the flag changes
  // nothing and the error does not say it.
  const std::string island = R"(, "minimum_reachability": 0)";
  for (const std::string& extra : {std::string(), kDepartAt, kArriveBy}) {
    const auto flagged =
        route_error(map, request({at("A"), at("x", kPrune + island)}, kNoPruning, extra), reader);
    ASSERT_TRUE(flagged) << extra;
    EXPECT_EQ(flagged->code, 442u) << extra;
    EXPECT_TRUE(flagged->pruned) << extra;
    EXPECT_NE(error_body(*flagged).find(R"("pruned":true)"), std::string::npos)
        << error_body(*flagged);
    const auto plain =
        route_error(map, request({at("A"), at("x", island)}, kNoPruning, extra), reader);
    ASSERT_TRUE(plain) << extra;
    EXPECT_EQ(plain->code, 442u) << extra;
    EXPECT_FALSE(plain->pruned) << extra;
    EXPECT_EQ(error_body(*plain).find("pruned"), std::string::npos) << error_body(*plain);
    const auto costing_prunes =
        route_error(map, request({at("A"), at("x", kPrune + island)}, "", extra), reader);
    ASSERT_TRUE(costing_prunes) << extra;
    EXPECT_FALSE(costing_prunes->pruned) << extra;
  }
}

TEST_F(PruneTest, ExhaustedPrunedPairSaysPruned) {
  for (const std::string& extra : {std::string(), kDepartAt, kArriveBy}) {
    const auto flagged = route_error(
        map, request({at("A"), at("F", kPrune)}, kNoPruning + ", " + cap(1), extra), reader);
    ASSERT_TRUE(flagged) << extra;
    EXPECT_EQ(flagged->code, 447u) << extra;
    EXPECT_TRUE(flagged->pruned) << extra;
    EXPECT_NE(error_body(*flagged).find(R"("pruned":true)"), std::string::npos);
    const auto plain =
        route_error(map, request({at("A"), at("F")}, kNoPruning + ", " + cap(1), extra), reader);
    ASSERT_TRUE(plain) << extra;
    EXPECT_EQ(plain->code, 447u) << extra;
    EXPECT_FALSE(plain->pruned) << extra;
    EXPECT_EQ(error_body(*plain).find("pruned"), std::string::npos);
  }
}

TEST_F(PruneTest, PrunedPairKeepsRelaxedRetry) {
  // P, halfway between E-F and the island road x-y, asks for heading south:
  // its one candidate is on the island, so the first pass to it fails; the
  // relaxed second pass (warning 401) adds the filtered E-F and relaxes the
  // pruned limits (here the user's), it does not drop them, and keeps their
  // default-limits flag (cleared: user limits). The unpruned pair runs
  // unlimited with the flag set; the next pruned pair begins with the stored
  // limits and their flag again. Bidirectional (x8 / x2) and, with depart_at,
  // unidirectional (x16 / x4).
  const auto& e = map.nodes.at("E");
  const auto& x = map.nodes.at("x");
  const midgard::PointLL p{x.lng(), (e.lat() + x.lat()) / 2};
  const std::string south =
      kPrune +
      R"(, "heading": 180, "heading_tolerance": 30, "radius": 60, "minimum_reachability": 0)";
  const std::string costing =
      kNoPruning +
      R"(, "hierarchy_limits": {"2": {"max_up_transitions": 1, "expand_within_distance": 150}})";
  struct Case {
    std::string extra;
    uint32_t relaxed_up;
    float relaxed_dist;
  };
  for (const auto& c : {Case{"", 8u, 300.f}, Case{kDepartAt, 16u, 600.f}}) {
    Engine engine(map, reader);
    engine.record_limits();
    Api api;
    engine.route(request({at("A"), loc(p, south), at("A"), at("F", kPrune)}, costing, c.extra), api);
    EXPECT_TRUE(warned(api, 401)) << c.extra;
    const auto legs = legs_of(api);
    ASSERT_EQ(legs.size(), 3u) << c.extra;
    EXPECT_EQ(legs[0].back(), "EF") << c.extra << ": the relaxed pass reached P on E-F";
    // first pass and relaxed pass to P, the unpruned pair, the last pair
    ASSERT_EQ(engine.thor.searches(), 4u) << c.extra;
    ASSERT_EQ(engine.seen.size(), 4u) << c.extra;
    EXPECT_EQ(engine.seen[1].max_up_transitions, 1u) << c.extra;
    EXPECT_FLOAT_EQ(engine.seen[1].expand_within_dist, 150.f) << c.extra;
    EXPECT_FALSE(engine.seen[1].default_limits) << c.extra;
    EXPECT_EQ(engine.seen[2].max_up_transitions, c.relaxed_up) << c.extra;
    EXPECT_FLOAT_EQ(engine.seen[2].expand_within_dist, c.relaxed_dist) << c.extra;
    EXPECT_FALSE(engine.seen[2].default_limits) << c.extra;
    EXPECT_EQ(engine.seen[3].max_up_transitions, kUnlimitedTransitions) << c.extra;
    EXPECT_TRUE(engine.seen[3].default_limits) << c.extra;
    EXPECT_EQ(engine.seen[4].max_up_transitions, 1u) << c.extra;
    EXPECT_FLOAT_EQ(engine.seen[4].expand_within_dist, 150.f) << c.extra;
    EXPECT_FALSE(engine.seen[4].default_limits) << c.extra;
  }
}

TEST_F(PruneTest, CustomLimitsAndTheirDefaultFlagFollowEachSearch) {
  // User limits (modification allowed) with pruning disabled: an unpruned
  // search runs unlimited with the default-limits flag set (as the costing
  // check leaves it), a pruned one with the user's limits and the flag
  // cleared -- each search with its own vector's flag, on both algorithms.
  const std::string costing =
      kNoPruning +
      R"(, "hierarchy_limits": {"2": {"max_up_transitions": 0, "expand_within_distance": 30}})";
  for (const std::string& extra : {std::string(), kDepartAt}) {
    Engine engine(map, reader);
    engine.record_limits();
    Api api;
    engine.route(request({at("A"), at("F"), at("A", kPrune), at("F")}, costing, extra), api);
    const auto legs = legs_of(api);
    ASSERT_EQ(legs.size(), 3u);
    EXPECT_EQ(legs[0], kShortCut) << extra;
    EXPECT_EQ(legs[1], reversed(kPrimaryRoad)) << extra;
    EXPECT_EQ(legs[2], kShortCut) << extra;
    ASSERT_EQ(engine.seen.size(), 3u) << extra;
    for (const uint32_t unpruned : {1u, 3u}) {
      EXPECT_EQ(engine.seen[unpruned].max_up_transitions, kUnlimitedTransitions) << extra;
      EXPECT_TRUE(engine.seen[unpruned].default_limits) << extra;
    }
    EXPECT_EQ(engine.seen[2].max_up_transitions, 0u) << extra;
    EXPECT_FLOAT_EQ(engine.seen[2].expand_within_dist, 30.f) << extra;
    EXPECT_FALSE(engine.seen[2].default_limits) << extra;
  }
}

TEST_F(PruneTest, PrunedFlagAppliesToLookaheadOfNextPair) {
  // A gate across C-D: the lookahead from it to F is a search of the next
  // pair, so it runs with F's (pruned, user) limits and their cleared
  // default-limits flag, like the leg to F itself; the leg to the gate runs
  // unlimited with the flag set.
  const auto& c = map.nodes.at("C");
  const auto& d = map.nodes.at("D");
  const midgard::PointLL mid{(c.lng() + d.lng()) / 2, c.lat()};
  const std::string costing =
      kNoPruning +
      R"(, "gate_lookahead": 2, "hierarchy_limits": {"2": {"max_up_transitions": 0, "expand_within_distance": 30}})";
  for (const std::string& extra : {std::string(), kDepartAt}) {
    Engine engine(map, reader);
    engine.record_limits();
    Api api;
    engine.route(request({at("A"), gate(mid, 90, 20), at("F", kPrune)}, costing, extra), api);
    // leg to the gate, lookahead from it, leg to F
    ASSERT_EQ(engine.seen.size(), 3u) << extra;
    EXPECT_EQ(engine.seen[1].max_up_transitions, kUnlimitedTransitions) << extra;
    EXPECT_TRUE(engine.seen[1].default_limits) << extra;
    for (const uint32_t pruned : {2u, 3u}) {
      EXPECT_EQ(engine.seen[pruned].max_up_transitions, 0u) << extra << " search " << pruned;
      EXPECT_FLOAT_EQ(engine.seen[pruned].expand_within_dist, 30.f) << extra << " search " << pruned;
      EXPECT_FALSE(engine.seen[pruned].default_limits) << extra << " search " << pruned;
    }
    EXPECT_EQ(legs_of(api).front(), (std::vector<std::string>{"AB", "BC", "CD", "CD", "DE", "EF"}))
        << extra;
  }
}

TEST_F(PruneTest, ThroughPointsKeepDirectionWithPrunedNeighbour) {
  // A through point on C-D, the pair after it pruned: one leg, on through the
  // point without turning back, the same maneuvers as without the flag.
  const auto& c = map.nodes.at("C");
  const auto& d = map.nodes.at("D");
  const midgard::PointLL mid{(c.lng() + d.lng()) / 2, c.lat()};
  const auto req = [&](const std::string& last) {
    return request({at("A"), loc(mid, kThrough), last}, kNoPruning);
  };
  std::string json;
  const auto pruned = route(req(at("F", kPrune)), &json);
  const auto plain = route(req(at("F")));
  ASSERT_EQ(pruned.trip().routes(0).legs_size(), 1);
  EXPECT_EQ(legs_of(pruned), legs_of(plain));
  EXPECT_NE(json.find(R"("pruned_pairs":[[1,2]])"), std::string::npos) << json;
  const auto& maneuvers = pruned.directions().routes(0).legs(0).maneuver();
  EXPECT_EQ(maneuvers.size(), plain.directions().routes(0).legs(0).maneuver_size());
  for (const auto& m : maneuvers) {
    EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnLeft);
    EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnRight);
  }
}

namespace {

// The same map with a 1 km culling limit: A-G is 0.4 km, G-F 1.2 km.
class CullingTest : public PruneTest {
protected:
  static gurka::map culled;
  static void SetUpTestSuite() {
    PruneTest::SetUpTestSuite();
    culled = map;
    culled.config.put("service_limits.max_distance_disable_hierarchy_culling", 1000);
  }
  static Api route_culled(const std::string& req, std::string* json = nullptr) {
    return gurka::do_action(Options::route, culled, req, reader, json);
  }
};
gurka::map CullingTest::culled = {};

} // namespace

TEST_F(CullingTest, PerPairHierarchyDistanceWithCap) {
  const auto& g = map.nodes.at("G");
  const auto& f = map.nodes.at("F");
  ASSERT_LT(map.nodes.at("A").Distance(g), 1000.);
  ASSERT_GT(g.Distance(f), 1000.);
  const std::string costing = kNoPruning + ", " + cap(1000000);
  std::string json;
  const auto result = route_culled(request({at("A"), at("G"), at("F")}, costing), &json);
  EXPECT_TRUE(warned(result, 205));
  // the costing keeps disable_hierarchy_pruning; only the long pair is pruned
  const auto& options = result.options();
  EXPECT_TRUE(
      options.costings().find(options.costing_type())->second.options().disable_hierarchy_pruning());
  EXPECT_FALSE(options.locations(1).prune_hierarchy());
  EXPECT_TRUE(options.locations(2).prune_hierarchy());
  EXPECT_NE(json.find(R"("pruned_pairs":[[1,2]])"), std::string::npos) << json;
  // the culled pair routes as one the client flagged itself
  const auto flagged = route(request({at("A"), at("G"), at("F", kPrune)}, costing));
  EXPECT_EQ(legs_of(result), legs_of(flagged));
}

TEST_F(CullingTest, NoPathOnACulledPairSaysPruned) {
  // A to the island is 1.2 km: with a cap loki prunes that pair itself, so
  // its 442 says pruned although the caller sent no flag. Without a cap the
  // whole request stops disabling pruning (as before): nothing to report.
  const auto& x = map.nodes.at("x");
  ASSERT_GT(map.nodes.at("A").Distance(x), 1000.);
  const std::string island = R"(, "minimum_reachability": 0)";
  const auto culled_pair =
      route_error(culled, request({at("A"), at("x", island)}, kNoPruning + ", " + cap(1000000)),
                  reader);
  ASSERT_TRUE(culled_pair);
  EXPECT_EQ(culled_pair->code, 442u);
  EXPECT_TRUE(culled_pair->pruned);
  EXPECT_NE(error_body(*culled_pair).find(R"("pruned":true)"), std::string::npos);
  const auto cumulative =
      route_error(culled, request({at("A"), at("x", island)}, kNoPruning), reader);
  ASSERT_TRUE(cumulative);
  EXPECT_EQ(cumulative->code, 442u);
  EXPECT_FALSE(cumulative->pruned);
}

TEST_F(CullingTest, WithoutCapCumulativeAsBefore) {
  std::string json;
  const auto result = route_culled(request({at("A"), at("G"), at("F")}, kNoPruning), &json);
  EXPECT_TRUE(warned(result, 205));
  const auto& options = result.options();
  EXPECT_FALSE(
      options.costings().find(options.costing_type())->second.options().disable_hierarchy_pruning());
  for (const auto& l : options.locations()) {
    EXPECT_FALSE(l.prune_hierarchy());
  }
  EXPECT_EQ(json.find("pruned_pairs"), std::string::npos) << json;
}

// ---------------------------------------------------------------------------
// The tile cache. A primary road across ~150 km and several level-0..2 tiles,
// residential stubs along it; a cache of one byte is always over its size.

namespace {

// A tile cache that counts the bytes it holds (the sizes the reader puts tiles
// in with) and logs every put and trim. Over its size when it holds more than
// `limit` bytes.
class CountingCache : public baldr::TileCache {
public:
  struct Event {
    char kind;        // 'P' put, 'T' trim, 'S' a search began (logged by the test)
    size_t held;      // bytes held after the event
    size_t tile_size; // the put tile's size
  };

  void Reserve(size_t) override {
  }
  bool Contains(const baldr::GraphId& id) const override {
    return tiles_.count(id) > 0;
  }
  baldr::graph_tile_ptr
  Put(const baldr::GraphId& id, baldr::graph_tile_ptr tile, size_t size) override {
    auto& slot = tiles_[id];
    held_ += size - slot.second;
    slot = {tile, size};
    if (log) {
      log->push_back({'P', held_, size});
    }
    return tile;
  }
  baldr::graph_tile_ptr Get(const baldr::GraphId& id) const override {
    const auto it = tiles_.find(id);
    return it == tiles_.end() ? nullptr : it->second.first;
  }
  bool OverCommitted() const override {
    return held_ > limit;
  }
  void Clear() override {
    tiles_.clear();
    held_ = 0;
  }
  void Trim() override {
    ++trims;
    Clear();
    if (log) {
      log->push_back({'T', held_, 0});
    }
  }
  size_t held_bytes() const {
    return held_;
  }
  size_t smallest_tile() const {
    size_t smallest = std::numeric_limits<size_t>::max();
    for (const auto& t : tiles_) {
      smallest = std::min(smallest, t.second.second);
    }
    return smallest;
  }

  size_t limit = std::numeric_limits<size_t>::max();
  size_t trims = 0;
  std::vector<Event>* log = nullptr;

private:
  std::unordered_map<baldr::GraphId, std::pair<baldr::graph_tile_ptr, size_t>> tiles_;
  size_t held_ = 0;
};

class CountingReader : public baldr::GraphReader {
public:
  CountingReader(const boost::property_tree::ptree& pt, size_t limit) : GraphReader(pt) {
    tile_extract_ = std::make_shared<baldr::GraphReader::tile_extract_t>(pt);
    auto counting = std::make_unique<CountingCache>();
    counting->limit = limit;
    cache = counting.get();
    cache_ = std::move(counting);
  }
  CountingCache* cache;
};

const std::string kLongMap = R"(
  A-----B-----C-----D-----E-----F
  |     |     |     |     |     |
  a     b     c     d     e     f
  )";

const gurka::ways kLongWays = {
    {"AB", {{"highway", "primary"}, {"name", "AB"}}},
    {"BC", {{"highway", "primary"}, {"name", "BC"}}},
    {"CD", {{"highway", "primary"}, {"name", "CD"}}},
    {"DE", {{"highway", "primary"}, {"name", "DE"}}},
    {"EF", {{"highway", "primary"}, {"name", "EF"}}},
    {"Aa", {{"highway", "residential"}, {"name", "Aa"}}},
    {"Bb", {{"highway", "residential"}, {"name", "Bb"}}},
    {"Cc", {{"highway", "residential"}, {"name", "Cc"}}},
    {"Dd", {{"highway", "residential"}, {"name", "Dd"}}},
    {"Ee", {{"highway", "residential"}, {"name", "Ee"}}},
    {"Ff", {{"highway", "residential"}, {"name", "Ff"}}},
};

std::vector<std::string> trip_names(const Api& api) {
  std::vector<std::string> names;
  for (const auto& leg : legs_of(api)) {
    names.insert(names.end(), leg.begin(), leg.end());
  }
  return names;
}

} // namespace

TEST(PairRecoveryTileCache, TileCacheTrimmedBetweenPairs) {
  // 5 km grid: each segment of the primary is 30 km, the road crosses several
  // level-2 (0.25 degree) tiles.
  const auto layout = gurka::detail::map_to_coordinates(kLongMap, 5000);
  const auto map =
      gurka::buildtiles(layout, kLongWays, {}, {}, VALHALLA_BUILD_DIR "test/data/pair_recovery_long");
  const auto& pt = map.config.get_child("mjolnir");
  const auto at = [&](const std::string& n, const std::string& extra = "") {
    return loc(map.nodes.at(n), extra);
  };
  // four pairs through three through points, assembled as one leg at the end
  for (const std::string& extra : {std::string(), kArriveBy}) {
    const auto req =
        request({at("a"), at("b", kThrough), at("d", kThrough), at("e", kThrough), at("f")}, "",
                extra);
    auto big = std::make_shared<CountingReader>(pt, std::numeric_limits<size_t>::max());
    Engine untrimmed(map, big);
    Api expected;
    untrimmed.route(req, expected);
    EXPECT_EQ(big->cache->trims, 0u);
    const size_t all_bytes = big->cache->held_bytes();
    // a real limit: the smallest tile of the trip
    const size_t limit = big->cache->smallest_tile();

    auto small = std::make_shared<CountingReader>(pt, limit);
    Engine trimmed(map, small);
    std::vector<CountingCache::Event> log;
    small->cache->log = &log;
    // a search's begin, with the trims before it
    std::vector<size_t> trims_at_search;
    uint32_t search = 0;
    trimmed.thor.track([&](baldr::GraphReader&, const baldr::GraphId, const baldr::GraphId,
                           const char*, const Expansion_EdgeStatus, float, uint32_t, float,
                           const Expansion_ExpansionType, const uint8_t, const TravelMode) {
      if (trimmed.thor.searches() != search) {
        search = trimmed.thor.searches();
        trims_at_search.push_back(small->cache->trims);
        log.push_back({'S', small->cache->held_bytes(), 0});
      }
    });
    Api api;
    trimmed.route(req, api);

    // the same trip
    EXPECT_EQ(trip_names(api), trip_names(expected)) << extra;
    ASSERT_EQ(api.trip().routes(0).legs_size(), 1);
    EXPECT_EQ(api.trip().routes(0).legs(0).shape(), expected.trip().routes(0).legs(0).shape());
    EXPECT_EQ(api.trip().routes(0).legs(0).node_size(),
              expected.trip().routes(0).legs(0).node_size());

    // trimmed between the pairs: each search after the first began after a trim
    ASSERT_EQ(trims_at_search.size(), 4u) << extra;
    for (size_t k = 1; k < trims_at_search.size(); ++k) {
      EXPECT_GT(trims_at_search[k], trims_at_search[k - 1]) << extra << " search " << k;
    }
    // Assembly: from the first trim after the last search began. The bytes the
    // cache holds never exceed its limit plus the tiles one edge reads (the
    // edge's, its end node's, a transition's: the 3 largest tiles touched).
    auto it =
        std::find_if(log.rbegin(), log.rend(), [](const auto& e) { return e.kind == 'S'; }).base();
    it = std::find_if(it, log.end(), [](const auto& e) { return e.kind == 'T'; });
    ASSERT_NE(it, log.end()) << extra << ": no trim before assembly";
    size_t assembly_max = 0;
    std::vector<size_t> sizes;
    for (; it != log.end(); ++it) {
      if (it->kind == 'P') {
        assembly_max = std::max(assembly_max, it->held);
        sizes.push_back(it->tile_size);
      }
    }
    ASSERT_GE(sizes.size(), 3u) << extra;
    std::sort(sizes.rbegin(), sizes.rend());
    const size_t bound = limit + sizes[0] + sizes[1] + sizes[2];
    EXPECT_LE(assembly_max, bound) << extra;
    // without trimming the whole trip's tiles stay, beyond that bound
    EXPECT_GT(all_bytes, bound) << extra;
  }
}
