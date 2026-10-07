// Gurka integration tests for the search label cap (vamoto patch 0049).
//
// A request's search_label_cap bounds every path search it makes, plain or
// loop, and is never raised. A search that crosses the cap ends with error
// 447 (never 442, never a route that is not proven best), at once: no relaxed
// second pass, no low-reachability retry, no gate lookahead reading it as "no
// way back". A search that runs out of memory is error 448.
//
// Most tests run on a 6x6 motorcycle grid at 100 m whose top row and right
// column are primary roads, so A to 9 has one best path.

#include "gurka.h"
#include "loki/worker.h"
#include "sif/costfactory.h"
#include "test.h"
#include "thor/bidirectional_astar.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <vector>

using namespace valhalla;

namespace {

std::string ll(const midgard::PointLL& p) {
  return R"("lon": )" + std::to_string(p.lng()) + R"(, "lat": )" + std::to_string(p.lat());
}

std::string loc(const midgard::PointLL& p, const std::string& extra = "") {
  return "{" + ll(p) + extra + "}";
}

std::string gate(const midgard::PointLL& p, int heading, int radius) {
  return loc(p, R"(, "type": "through", "gate_heading": )" + std::to_string(heading) +
                    R"(, "gate_radius": )" + std::to_string(radius) +
                    R"(, "minimum_reachability": 0)");
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

// The code a request fails with, 0 when it routes.
unsigned route_code(const gurka::map& map,
                    const std::string& req,
                    valhalla::Api* out = nullptr,
                    std::shared_ptr<baldr::GraphReader> reader = {}) {
  try {
    auto result = gurka::do_action(Options::route, map, req, reader);
    if (out) {
      *out = result;
    }
    return 0;
  } catch (const valhalla_exception_t& e) { return e.code; }
}

std::vector<std::string> path_of(const valhalla::Api& result) {
  return gurka::detail::get_paths(result).front();
}

// ---------------------------------------------------------------------------
// The grid.

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
  return ways;
}

const std::vector<std::string> kBestPath = {"AB", "BC", "CD", "DE", "EF",
                                            "FL", "LR", "RX", "X3", "39"};

class SearchLabelCapTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(grid_ascii(), 50);
    map = gurka::buildtiles(layout, grid_ways(), {}, {},
                            VALHALLA_BUILD_DIR "test/data/search_label_cap");
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
  static void TearDownTestSuite() {
    reader.reset();
  }

  static std::string a_to_9(const std::string& costing = "", const std::string& extra = "") {
    return request({loc(map.nodes.at("A")), loc(map.nodes.at("9"))}, costing, extra);
  }

  // The smallest cap A to 9 routes with, at most `limit`.
  static uint32_t smallest_routing_cap(const std::string& extra_costing = "",
                                       const std::string& extra = "",
                                       uint32_t limit = 20000) {
    for (uint32_t c = 1; c <= limit; ++c) {
      if (route_code(map, a_to_9(cap(c) + extra_costing, extra), nullptr, reader) == 0) {
        return c;
      }
    }
    return 0;
  }
};
gurka::map SearchLabelCapTest::map = {};
std::shared_ptr<baldr::GraphReader> SearchLabelCapTest::reader = {};

const std::string kDepartAt = R"(, "date_time": {"type": 1, "value": "2025-08-15T14:50"})";
const std::string kArriveBy = R"(, "date_time": {"type": 2, "value": "2025-08-15T14:50"})";

} // namespace

TEST_F(SearchLabelCapTest, PlainWithoutCapRoutesAsBefore) {
  auto result = gurka::do_action(Options::route, map, a_to_9(), reader);
  gurka::assert::raw::expect_path(result, kBestPath);
  EXPECT_EQ(result.info().warnings_size(), 0);
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
}

TEST_F(SearchLabelCapTest, PlainWithAmpleCapRoutesTheSamePath) {
  auto result = gurka::do_action(Options::route, map, a_to_9(cap(1000000)), reader);
  gurka::assert::raw::expect_path(result, kBestPath);
  EXPECT_EQ(result.info().warnings_size(), 0);
}

TEST_F(SearchLabelCapTest, TinyCapIs447NotNoPath) {
  try {
    gurka::do_action(Options::route, map, a_to_9(cap(1)), reader);
    FAIL() << "a 1-label search must not route";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 447u);
    EXPECT_EQ(e.http_code, 400u);
    EXPECT_EQ(e.statsd_key, "search_budget_exhausted");
  }
}

TEST_F(SearchLabelCapTest, SmallCapIsNotRaised) {
  // The old 50k floor routed this whole map at any cap.
  EXPECT_EQ(route_code(map, a_to_9(cap(10)), nullptr, reader), 447u);
}

TEST_F(SearchLabelCapTest, ExhaustedWithAConnectionIsStill447) {
  // For every cap up to a little past the smallest routing one: the expansion
  // (which swallows the error) shows whether the search had connected the two
  // trees; the route either is the best path or 447, never another path.
  const uint32_t c_min = smallest_routing_cap();
  ASSERT_GT(c_min, 1u);
  bool connected_but_447 = false;
  for (uint32_t c = 1; c <= c_min + 20; ++c) {
    const auto req = a_to_9(cap(c), R"(, "action": "route", "format": "pbf",)"
                                    R"( "expansion_properties": ["edge_status"])");
    std::string pbf;
    gurka::do_action(Options::expansion, map, req, reader, &pbf);
    Api expansion;
    ASSERT_TRUE(expansion.ParseFromString(pbf));
    const auto& statuses = expansion.expansion().edge_status();
    const bool connected = std::any_of(statuses.begin(), statuses.end(), [](int s) {
      return s == Expansion_EdgeStatus_connected;
    });

    valhalla::Api result;
    const auto code = route_code(map, a_to_9(cap(c)), &result, reader);
    if (code == 0) {
      EXPECT_EQ(path_of(result), kBestPath) << "cap " << c;
    } else {
      EXPECT_EQ(code, 447u) << "cap " << c;
      connected_but_447 = connected_but_447 || connected;
    }
  }
  EXPECT_TRUE(connected_but_447) << "no cap stopped a search that had a connection";
}

TEST_F(SearchLabelCapTest, ExhaustionSkipsTheRelaxedRetry) {
  tyr::actor_t actor(map.config, *reader, true);
  Api api;
  try {
    actor.route(a_to_9(cap(1)), nullptr, &api);
    FAIL() << "a 1-label search must not route";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 447u); }
  for (const auto& w : api.info().warnings()) {
    EXPECT_NE(w.code(), 401u) << "the relaxed second pass ran";
  }
}

TEST_F(SearchLabelCapTest, DepartAtTimedepForwardExhaustionIs447) {
  valhalla::Api result;
  ASSERT_EQ(route_code(map, a_to_9(cap(1000000), kDepartAt), &result, reader), 0u);
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_forward_a*");
  EXPECT_EQ(path_of(result), kBestPath);
  EXPECT_EQ(route_code(map, a_to_9(cap(1), kDepartAt), nullptr, reader), 447u);
}

TEST_F(SearchLabelCapTest, ArriveByTimedepReverseExhaustionIs447) {
  valhalla::Api result;
  ASSERT_EQ(route_code(map, a_to_9(cap(1000000), kArriveBy), &result, reader), 0u);
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "time_dependent_reverse_a*");
  EXPECT_EQ(path_of(result), kBestPath);
  EXPECT_EQ(route_code(map, a_to_9(cap(1), kArriveBy), nullptr, reader), 447u);
}

TEST_F(SearchLabelCapTest, ArriveByBidirectionalExhaustionIs447) {
  auto far = map;
  far.config.put("service_limits.max_timedep_distance", 1);
  valhalla::Api result;
  ASSERT_EQ(route_code(far, a_to_9(cap(1000000), kArriveBy), &result, reader), 0u);
  EXPECT_EQ(result.trip().routes(0).legs(0).algorithms(0), "bidirectional_a*");
  EXPECT_EQ(path_of(result), kBestPath);
  EXPECT_EQ(route_code(far, a_to_9(cap(1), kArriveBy), nullptr, reader), 447u);
}

TEST_F(SearchLabelCapTest, LoopDefaultCapExhaustionIs447) {
  // A loop request that sends no cap gets the loop default; exhausting it is
  // 447 like any cap. The default is lowered so this map can exhaust it.
  struct RestoreDefault {
    ~RestoreDefault() {
      sif::DynamicCost::loop_search_label_cap_default = sif::DynamicCost::kLoopSearchLabelCap;
    }
  } restore;
  sif::DynamicCost::loop_search_label_cap_default = 5;
  EXPECT_EQ(route_code(map, a_to_9(R"("reuse_factor": 4)"), nullptr, reader), 447u);
  // A plain request without a cap is unbounded as before.
  valhalla::Api result;
  ASSERT_EQ(route_code(map, a_to_9(), &result, reader), 0u);
  EXPECT_EQ(path_of(result), kBestPath);
}

namespace {

// As in gurka_bidir_search (exhaust_reverse_search): the slow roads round the
// start are marked not_thru, so without the extended search the first
// bidirectional pass fails at once; the relaxed pass (warning 401) stops
// pruning, and searches the slow grid before it gets out by D.
const std::string kNotThruMap = R"(
      A---B---C---D-E-F
      |   |   |
      G---H---I
      |   |   |
      J---K---L
  )";

const gurka::ways kNotThruWays = {
    {"AB", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"BC", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"CD", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"AG", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"BH", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"CI", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"GH", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"HI", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"GJ", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"HK", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"IL", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"JK", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"KL", {{"highway", "secondary"}, {"maxspeed", "10"}}},
    {"DE", {{"highway", "motorway"}}},
    {"EF", {{"highway", "motorway"}}},
};

bool warned_401(const Api& api) {
  return std::any_of(api.info().warnings().begin(), api.info().warnings().end(),
                     [](const auto& w) { return w.code() == 401u; });
}

} // namespace

TEST(SearchLabelCapRelaxed, ExhaustedRelaxedPassIs447) {
  const auto layout = gurka::detail::map_to_coordinates(kNotThruMap, 100);
  auto map = gurka::buildtiles(layout, kNotThruWays, {}, {},
                               VALHALLA_BUILD_DIR "test/data/search_label_cap_not_thru");
  {
    auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
    std::vector<baldr::GraphId> not_thru;
    for (const auto& [from, to] : std::vector<std::pair<std::string, std::string>>{
             {"A", "B"}, {"B", "C"}, {"C", "D"}, {"A", "G"}, {"B", "H"}, {"C", "I"},
             {"G", "H"}, {"H", "I"}, {"G", "J"}, {"H", "K"}, {"I", "L"}, {"J", "K"},
             {"K", "L"}}) {
      not_thru.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, from, to)));
      not_thru.push_back(std::get<0>(gurka::findEdgeByNodes(*reader, layout, to, from)));
    }
    test::customize_edges(map.config,
                          [&not_thru](const baldr::GraphId& id, baldr::DirectedEdge& e) {
                            if (std::find(not_thru.begin(), not_thru.end(), id) != not_thru.end()) {
                              e.set_not_thru(true);
                            }
                          });
  }
  map.config.put("thor.extended_search", false);
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto req = [&](const std::string& costing) {
    return request({loc(map.nodes.at("A")), loc(map.nodes.at("F"))}, costing);
  };
  tyr::actor_t actor(map.config, *reader, true);
  {
    Api api;
    actor.route(req(""), nullptr, &api);
    ASSERT_TRUE(warned_401(api)) << "the first pass must fail for this test to mean anything";
  }
  // Some cap lets the first pass fail on its own (no path) and stops the
  // relaxed pass: 447, after the 401 warning.
  bool relaxed_exhausted = false;
  for (uint32_t c = 1; c <= 2000 && !relaxed_exhausted; ++c) {
    Api api;
    try {
      actor.route(req(cap(c)), nullptr, &api);
      break; // routes from here on
    } catch (const valhalla_exception_t& e) {
      ASSERT_EQ(e.code, 447u) << "cap " << c;
      relaxed_exhausted = warned_401(api);
    }
  }
  EXPECT_TRUE(relaxed_exhausted) << "no cap stopped the relaxed pass";
}

// ---------------------------------------------------------------------------
// Workers driven directly: one thor worker across requests, and a costing
// that runs out of memory.

namespace {

// A costing whose every edge cost throws std::bad_alloc, as a search that
// runs out of memory does.
class BadAllocCost final : public sif::DynamicCost {
public:
  explicit BadAllocCost(const Costing& options)
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
    throw std::bad_alloc();
  }
  sif::Cost EdgeCost(const baldr::DirectedEdge*,
                     const baldr::GraphId&,
                     const baldr::graph_tile_ptr&,
                     const baldr::TimeInfo&,
                     uint8_t&) const override {
    throw std::bad_alloc();
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
  bool Allowed(const baldr::DirectedEdge* edge,
               const baldr::graph_tile_ptr&,
               uint16_t) const override {
    return !edge->is_shortcut();
  }
};

class TestThor : public thor::thor_worker_t {
public:
  using thor::thor_worker_t::thor_worker_t;
  void search_runs_out_of_memory() {
    factory.Register(Costing::motorcycle, [](const Costing& options) -> sif::cost_ptr_t {
      return std::make_shared<BadAllocCost>(options);
    });
  }
  void track(const thor::PathAlgorithm::expansion_callback_t& callback) {
    bidir_astar.set_track_expansion(callback);
    timedep_forward.set_track_expansion(callback);
    timedep_reverse.set_track_expansion(callback);
  }
  // GetBestPath calls of the last route request.
  uint32_t searches() const {
    return path_searches_;
  }
  bool any_exhausted() const {
    return bidir_astar.search_budget_exhausted() || timedep_forward.search_budget_exhausted() ||
           timedep_reverse.search_budget_exhausted();
  }
};

// loki + thor on one map, without cleanup between requests (the workers'
// state carries over, as on a busy service).
struct Engine {
  std::shared_ptr<baldr::GraphReader> reader;
  loki::loki_worker_t loki;
  TestThor thor;
  explicit Engine(const gurka::map& map)
      : reader(test::make_clean_graphreader(map.config.get_child("mjolnir"))),
        loki(map.config, reader), thor(map.config, reader) {
  }
  // The code the request fails with in thor, 0 when it routes.
  unsigned route(const std::string& req, Api& api) {
    api.Clear();
    ParseApi(req, Options::route, api);
    loki.route(api);
    try {
      thor.route(api);
    } catch (const valhalla_exception_t& e) { return e.code; }
    return 0;
  }
};

} // namespace

TEST_F(SearchLabelCapTest, OutOfMemorySearchIs448Not499) {
  Engine engine(map);
  engine.thor.search_runs_out_of_memory();
  Api api;
  try {
    ParseApi(a_to_9(), Options::route, api);
    engine.loki.route(api);
    engine.thor.route(api);
    FAIL() << "the search must run out of memory";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 448u);
    EXPECT_EQ(e.http_code, 503u);
    EXPECT_EQ(e.statsd_key, "search_allocation_failure");
    // What a caller gets back.
    const auto json = serialize_error(e, api);
    EXPECT_NE(json.find(R"("error_code":448)"), std::string::npos) << json;
    EXPECT_NE(json.find(R"("status_code":503)"), std::string::npos) << json;
  }
}

TEST(SearchErrorCodes, BudgetAndMemoryCodesAreDistinctFromNoPath) {
  const valhalla_exception_t no_path{442}, exhausted{447}, out_of_memory{448};
  EXPECT_EQ(exhausted.code, 447u);
  EXPECT_EQ(exhausted.http_code, 400u);
  EXPECT_EQ(exhausted.statsd_key, "search_budget_exhausted");
  EXPECT_EQ(out_of_memory.code, 448u);
  EXPECT_EQ(out_of_memory.http_code, 503u);
  EXPECT_EQ(out_of_memory.statsd_key, "search_allocation_failure");
  EXPECT_NE(exhausted.message, no_path.message);
  EXPECT_NE(out_of_memory.message, no_path.message);
  EXPECT_NE(exhausted.message, out_of_memory.message);
}

TEST_F(SearchLabelCapTest, AnExhaustedSearchLeavesNoTraceOnTheNext) {
  // S3: one thor worker, no cleanup: an exhausted search, then searches that
  // route, on the bidirectional and the time-dependent algorithm.
  Engine engine(map);
  Api api;
  for (const std::string& extra : {std::string(), kDepartAt}) {
    EXPECT_EQ(engine.route(a_to_9(cap(1), extra), api), 447u);
    EXPECT_TRUE(engine.thor.any_exhausted());
    ASSERT_EQ(engine.route(a_to_9("", extra), api), 0u);
    EXPECT_FALSE(engine.thor.any_exhausted());
    ASSERT_EQ(engine.route(a_to_9(cap(1000000), extra), api), 0u);
    EXPECT_FALSE(engine.thor.any_exhausted());
  }
}

// ---------------------------------------------------------------------------
// Label reservations follow the cap.

namespace {

sif::cost_ptr_t motorcycle_with(const std::string& costing) {
  Api api;
  ParseApi(request({R"({"lon": 0, "lat": 0})", R"({"lon": 0.01, "lat": 0.01})"}, costing),
           Options::route, api);
  return sif::CostFactory().Create(api.options());
}

boost::property_tree::ptree reserve_config() {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_bidir_astar", 10000);
  config.put("max_reserved_labels_count_astar", 10000);
  return config;
}

class BidirProbe : public thor::BidirectionalAStar {
public:
  BidirProbe() : BidirectionalAStar(reserve_config()) {
  }
  size_t reserve_for(const sif::cost_ptr_t& costing) {
    costing_ = costing;
    Init({0, 0}, {0.01, 0.01});
    return std::min(edgelabels_forward_.capacity(), edgelabels_reverse_.capacity());
  }
  uint32_t label_cap() const {
    return label_cap_;
  }
  static constexpr uint32_t kSlack = kLabelCapSlack;
};

class UnidirProbe : public thor::TimeDepForward {
public:
  UnidirProbe() : UnidirectionalAStar(reserve_config()) {
  }
  size_t reserve_for(const sif::cost_ptr_t& costing) {
    costing_ = costing;
    Init({0, 0}, {0.01, 0.01});
    return edgelabels_.capacity();
  }
  static constexpr uint32_t kSlack = kLabelCapSlack;
};

} // namespace

TEST(SearchLabelReserve, BidirReservationFollowsTheCap) {
  EXPECT_GE(BidirProbe().reserve_for(motorcycle_with("")), 10000u);
  const auto capped = BidirProbe().reserve_for(motorcycle_with(cap(100)));
  EXPECT_LE(capped, 100u + BidirProbe::kSlack);
  EXPECT_LT(capped, 10000u);
}

TEST(SearchLabelReserve, UnidirReservationFollowsTheCap) {
  EXPECT_GE(UnidirProbe().reserve_for(motorcycle_with("")), 10000u);
  const auto capped = UnidirProbe().reserve_for(motorcycle_with(cap(100)));
  EXPECT_LE(capped, 100u + UnidirProbe::kSlack);
  EXPECT_LT(capped, 10000u);
}

TEST(SearchLabelReserve, ReservationFollowsTheCurrentCapNotThePrevious) {
  // S3: one instance, a small cap and then a larger one: the second search
  // reserves for its own cap.
  BidirProbe probe;
  EXPECT_LE(probe.reserve_for(motorcycle_with(cap(100))), 100u + BidirProbe::kSlack);
  EXPECT_EQ(probe.label_cap(), 100u);
  EXPECT_GE(probe.reserve_for(motorcycle_with(cap(8000))), 10000u);
  EXPECT_EQ(probe.label_cap(), 8000u);
  probe.Clear();
  EXPECT_GE(probe.reserve_for(motorcycle_with("")), 10000u);
  EXPECT_EQ(probe.label_cap(), 0u);
}

// ---------------------------------------------------------------------------
// What the bidirectional search would have done next. A search that crosses
// its cap returns at once, so its state after GetBestPath is its state at the
// cap check: a probe reads it there.

namespace {

class BidirStateProbe : public thor::BidirectionalAStar {
public:
  explicit BidirStateProbe(const boost::property_tree::ptree& thor_config)
      : BidirectionalAStar(thor_config) {
    // The direction of the last expansion is the direction the next
    // iteration pops (a pop that only records a connection keeps it).
    set_track_expansion([this](baldr::GraphReader&, const baldr::GraphId, const baldr::GraphId,
                               const char*, const Expansion_EdgeStatus status, float, uint32_t,
                               float, const Expansion_ExpansionType type, const uint8_t,
                               const TravelMode) {
      if (status == Expansion_EdgeStatus_settled) {
        last_expanded_forward_ = type == Expansion_ExpansionType_forward;
      }
    });
  }
  uint32_t labels() const {
    return edgelabels_forward_.size() + edgelabels_reverse_.size();
  }
  bool past_iterations_threshold() const {
    return labels() > iterations_threshold_;
  }
  bool connected() const {
    return !best_connections_.empty();
  }
  // Would the next pop -- the one the cap check pre-empted -- have ended the
  // search normally: on the cost threshold, or on an empty frontier after a
  // connection? It pops that queue, so call it once, after the search.
  bool next_pop_ends_normally() {
    if (!last_expanded_forward_) {
      return false;
    }
    const auto ends = [this](auto& queue, const auto& labels, float diff) {
      const auto idx = queue.pop();
      if (idx == baldr::kInvalidLabel) {
        return !best_connections_.empty();
      }
      return labels[idx].sortcost() + diff > cost_threshold_;
    };
    return *last_expanded_forward_
               ? ends(adjacencylist_forward_, edgelabels_forward_, cost_diff_)
               : ends(adjacencylist_reverse_, edgelabels_reverse_, 0.0f);
  }

private:
  std::optional<bool> last_expanded_forward_;
};

// A route request located by loki once, to search again with any cap.
struct Located {
  Api api;
  Located(const gurka::map& map,
          const std::shared_ptr<baldr::GraphReader>& reader,
          const std::string& req) {
    ParseApi(req, Options::route, api);
    loki::loki_worker_t(map.config, reader).route(api);
  }
  // One bidirectional search with `probe` and label cap `cap`, as thor's
  // first pass runs it; true when it returned a path.
  bool search(BidirStateProbe& probe, baldr::GraphReader& reader, uint32_t cap) const {
    Api copy = api;
    auto& costings = *copy.mutable_options()->mutable_costings();
    costings.at(Costing::motorcycle).mutable_options()->set_search_label_cap(cap);
    sif::TravelMode mode;
    auto mode_costing = sif::CostFactory().CreateModeCosting(copy.options(), mode);
    mode_costing[static_cast<uint32_t>(mode)]->set_allow_destination_only(false);
    auto& locations = *copy.mutable_options()->mutable_locations();
    return !probe
                .GetBestPath(*locations.Mutable(0), *locations.Mutable(1), reader, mode_costing,
                             mode, copy.options())
                .empty();
  }
};

} // namespace

TEST_F(SearchLabelCapTest, CrossingTheCapBeatsANormalEnd) {
  // S5 precedence: a search whose cap is crossed right before the pop that
  // would have ended it on its cost threshold still ends in 447. With no
  // threshold extension (threshold_delta 0) the search ends on the first pop
  // past its best connection; swept over A to every node and every cap until
  // the cap falls right before that pop.
  auto tight = map;
  tight.config.put("thor.bidirectional_astar.threshold_delta", 0);
  // (destination, cap, labels when the cap was crossed)
  std::optional<std::tuple<std::string, uint32_t, uint32_t>> witness;
  for (const auto& row : kRows) {
    for (const char to : row) {
      if (to == 'A' || witness) {
        continue;
      }
      const auto& dest = tight.nodes.at(std::string(1, to));
      const Located located(tight, reader, request({loc(tight.nodes.at("A")), loc(dest)}, cap(0)));
      for (uint32_t c = 1; c <= 5000; ++c) {
        BidirStateProbe probe(tight.config.get_child("thor"));
        if (located.search(probe, *reader, c)) {
          break;
        }
        ASSERT_TRUE(probe.search_budget_exhausted()) << to << " cap " << c;
        if (probe.connected() && probe.next_pop_ends_normally()) {
          witness = {std::string(1, to), c, probe.labels()};
          break;
        }
      }
    }
  }
  ASSERT_TRUE(witness) << "no cap was crossed right before a normal end";
  const auto& [to, c, labels] = *witness;
  const auto req = [&](uint32_t c) {
    return request({loc(tight.nodes.at("A")), loc(tight.nodes.at(to))}, cap(c));
  };
  EXPECT_EQ(route_code(tight, req(c), nullptr, reader), 447u) << to << " cap " << c;
  // With room for exactly the labels it held then, the same search makes
  // that pop and routes.
  const Located located(tight, reader, req(0));
  BidirStateProbe probe(tight.config.get_child("thor"));
  EXPECT_TRUE(located.search(probe, *reader, labels)) << to << " cap " << labels;
  EXPECT_FALSE(probe.search_budget_exhausted());
}

TEST_F(SearchLabelCapTest, CapCrossedWithTheAlternativesThresholdIs447) {
  // An alternatives request ends on its iterations threshold, alternatives'
  // iterations after the first connection. A cap crossed in the same
  // expansion wins: 447, never the route found so far.
  auto alt = map;
  alt.config.put("thor.bidirectional_astar.alternative_iterations_delta", 1);
  const std::string alternates = R"(, "alternates": 1)";
  const Located located(alt, reader, a_to_9(cap(0), alternates));
  std::optional<uint32_t> witness;
  for (uint32_t c = 1; c <= 5000 && !witness; ++c) {
    BidirStateProbe probe(alt.config.get_child("thor"));
    if (located.search(probe, *reader, c)) {
      break;
    }
    ASSERT_TRUE(probe.search_budget_exhausted()) << "cap " << c;
    if (probe.connected() && probe.past_iterations_threshold()) {
      witness = c;
    }
  }
  ASSERT_TRUE(witness) << "no cap was crossed together with the iterations threshold";
  EXPECT_EQ(route_code(alt, a_to_9(cap(*witness), alternates), nullptr, reader), 447u);
  valhalla::Api result;
  ASSERT_EQ(route_code(alt, a_to_9(cap(1000000), alternates), &result, reader), 0u);
  EXPECT_EQ(path_of(result), kBestPath);
}

// ---------------------------------------------------------------------------
// Gate lookahead (patch 0033): an exhausted `ahead` or `again` search is 447,
// never "no way back" and never the leg it was checking.

namespace {

// The forward searches a request ran, in order, by the class of edge each was
// seeded on: a settled forward label without a predecessor is a seed; one on
// an edge of `origin_edges` is 'O', on any other edge 'X'; repeats collapse.
// (A callback's second edge is the predecessor of the label it expands from,
// so only a settled event's names the event's own predecessor.)
struct SearchTrace {
  std::vector<baldr::GraphId> origin_edges;
  std::string seeds;
  thor::PathAlgorithm::expansion_callback_t callback() {
    return [this](baldr::GraphReader&, const baldr::GraphId edge, const baldr::GraphId prev,
                  const char*, const Expansion_EdgeStatus status, float, uint32_t, float,
                  const Expansion_ExpansionType type, const uint8_t, const TravelMode) {
      if (type != Expansion_ExpansionType_forward || status != Expansion_EdgeStatus_settled ||
          prev.is_valid()) {
        return;
      }
      const bool o =
          std::find(origin_edges.begin(), origin_edges.end(), edge) != origin_edges.end();
      const char c = o ? 'O' : 'X';
      if (seeds.empty() || seeds.back() != c) {
        seeds += c;
      }
    };
  }
};

// Both directions of every edge between `node` and its `neighbours`.
std::vector<baldr::GraphId> edges_at(baldr::GraphReader& reader,
                                     const gurka::nodelayout& nodes,
                                     const std::string& node,
                                     const std::vector<std::string>& neighbours) {
  std::vector<baldr::GraphId> ids;
  for (const auto& n : neighbours) {
    ids.push_back(std::get<0>(gurka::findEdgeByNodes(reader, nodes, node, n)));
    ids.push_back(std::get<0>(gurka::findEdgeByNodes(reader, nodes, n, node)));
  }
  return ids;
}

struct LookaheadWitness {
  uint32_t cap = 0;
  std::string seeds;
};

// Sweep the cap: the first cap where the request fails 447 after exactly
// want.size() path searches whose forward searches were seeded in the order
// `want`. One letter per search means every search begun got to its first
// pop, so the last one is the one that exhausted -- and the next leg never
// began. Every cap's outcome is a route or 447, nothing else.
std::optional<LookaheadWitness> sweep_for(const gurka::map& map,
                                          const std::vector<baldr::GraphId>& origin_edges,
                                          const std::function<std::string(uint32_t)>& req,
                                          const std::string& want,
                                          uint32_t limit = 5000) {
  Engine engine(map);
  SearchTrace trace{origin_edges, ""};
  engine.thor.track(trace.callback());
  Api api;
  for (uint32_t c = 1; c <= limit; ++c) {
    trace.seeds.clear();
    const auto code = engine.route(req(c), api);
    EXPECT_TRUE(code == 0 || code == 447u) << "cap " << c << " failed " << code;
    if (code == 0) {
      break;
    }
    if (engine.thor.searches() == want.size() && trace.seeds == want) {
      return LookaheadWitness{c, trace.seeds};
    }
  }
  return std::nullopt;
}

} // namespace

TEST_F(SearchLabelCapTest, ExhaustedLookaheadAheadIs447) {
  // A gate across B-C right after the start, the destination far across the
  // grid: the first leg is short, the `ahead` search from its crossing long.
  const auto& b = map.nodes.at("B");
  const auto& c = map.nodes.at("C");
  const midgard::PointLL mid{(b.lng() + c.lng()) / 2, b.lat()};
  const auto req = [&](uint32_t c) {
    return request({loc(map.nodes.at("A")), gate(mid, 90, 20), loc(map.nodes.at("9"))},
                   R"("gate_lookahead": 2, )" + cap(c));
  };
  Engine probe(map);
  Api api;
  ASSERT_EQ(probe.route(req(1000000), api), 0u);
  // leg, ahead, leg: the ahead search is the second of three.
  ASSERT_EQ(probe.thor.searches(), 3u);

  const auto origin_edges = edges_at(*reader, map.nodes, "A", {"B", "G"});
  // O = the first leg from A, X = the `ahead` search from the B-C crossing.
  const auto witness = sweep_for(map, origin_edges, req, "OX");
  ASSERT_TRUE(witness) << "no cap let the first leg route and stopped the ahead search";
  // Asserted on its own: that cap is 447.
  EXPECT_EQ(Engine(map).route(req(witness->cap), api), 447u);
}

namespace {

// P just below A, a gate across A-B and, far below, across D-E. From P the
// first leg crosses A-B; the `ahead` search from there back to P can only go
// round by E, X and W and ride A-P back (a retrace), so the leg is routed
// `again`, to the D-E crossing: the long way round by W and X, with the grid
// below P, which leads nowhere, in its way -- a far larger search than the
// first leg or its ahead search.
const std::string kAgainMap = R"(
      W-----------A---B
      |           |   |
      |           P   |
      |           |   |
      |   a-b-c-d-e   |
      |   | | | | |   |
      |   f-g-h-i-j   |
      |   | | | | |   |
      |   k-l-m-n-o   |
      |   | | | | |   |
      |   q-r-s-t-u   |
      |               |
      X-----------D---E
  )";

gurka::ways again_ways() {
  gurka::ways ways;
  for (const std::string name : {"WA", "AB", "AP", "Pe", "WX", "XD", "DE", "BE"}) {
    ways[name] = {{"highway", "secondary"}, {"name", name}};
  }
  const std::vector<std::string> rows = {"abcde", "fghij", "klmno", "qrstu"};
  for (size_t r = 0; r < rows.size(); ++r) {
    for (size_t c = 0; c < rows[r].size(); ++c) {
      if (c + 1 < rows[r].size()) {
        const std::string name = {rows[r][c], rows[r][c + 1]};
        ways[name] = {{"highway", "residential"}, {"name", name}};
      }
      if (r + 1 < rows.size()) {
        const std::string name = {rows[r][c], rows[r + 1][c]};
        ways[name] = {{"highway", "residential"}, {"name", name}};
      }
    }
  }
  return ways;
}

} // namespace

TEST(SearchLabelCapLookahead, ExhaustedLookaheadAgainIs447) {
  const auto layout = gurka::detail::map_to_coordinates(kAgainMap, 50);
  const auto map = gurka::buildtiles(layout, again_ways(), {}, {},
                                     VALHALLA_BUILD_DIR "test/data/search_label_cap_again");
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  const auto& a = map.nodes.at("A");
  const auto& b = map.nodes.at("B");
  const auto& d = map.nodes.at("D");
  // Halfway along A-B, halfway down to D-E; A-B and D-E are 300 m off, the
  // grid below P lies wholly west of the gate.
  const midgard::PointLL mid{(a.lng() + b.lng()) / 2, (a.lat() + d.lat()) / 2};
  const auto req = [&](uint32_t c) {
    return request({loc(map.nodes.at("P")), gate(mid, 90, 320), loc(map.nodes.at("P"))},
                   R"("gate_lookahead": 2, )" + cap(c));
  };
  Engine probe(map);
  Api api;
  ASSERT_EQ(probe.route(req(1000000), api), 0u);

  const auto origin_edges = edges_at(*reader, layout, "P", {"A", "e"});
  // O = the first leg from P, X = the `ahead` search from the A-B crossing,
  // O = the leg routed `again`.
  const auto witness = sweep_for(map, origin_edges, req, "OXO");
  ASSERT_TRUE(witness) << "no cap let the first leg and its ahead search finish and stopped again";
  EXPECT_EQ(Engine(map).route(req(witness->cap), api), 447u);
}
