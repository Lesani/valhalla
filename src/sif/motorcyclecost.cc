#include "sif/motorcyclecost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/osrm_car_duration.h"
#include "sif/scenic_cost_helpers.h"

#ifdef INLINE_TEST
#include "test.h"
#include "worker.h"

#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Other options
constexpr float kDefaultUseHighways = 0.5f; // Factor between 0 and 1
constexpr float kDefaultUseTolls = 0.5f;    // Factor between 0 and 1
constexpr float kDefaultUseTrails = 0.0f;   // Factor between 0 and 1

// better_mc_routing: ETA credibility floor. Valhalla bakes walking-pace default
// speeds (2-12 km/h) onto track/path/driveway edges. A road-going motorcycle does
// not crawl at that speed, so a mandatory low-class connector (e.g. an endpoint
// snapped onto a forest track) would otherwise inflate the reported ETA 3-4x.
// Floor the speed used for ELAPSED TIME at this value. Elapsed time only — the
// cost-shaping factors (surface/class/curve) are untouched, so route choice on
// real roads is unchanged. Ferries are exempt (their speed is legitimately low).
constexpr uint32_t kMotorcycleMinEtaSpeed = 25; // km/h

constexpr Surface kMinimumMotorcycleSurface = Surface::kImpassable;

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;
constexpr float kTCRamp = 1.5f;
constexpr float kTCRoundabout = 0.5f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Valid ranges and defaults
constexpr ranged_default_t<float> kUseHighwaysRange{0, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};
constexpr ranged_default_t<float> kUseTrailsRange{0, kDefaultUseTrails, 1.0f};
constexpr ranged_default_t<uint32_t> kMotorcycleSpeedRange{10, baldr::kMaxAssumedSpeed,
                                                           baldr::kMaxSpeedKph};

// better_mc_routing v1: motorcycle_curvy option ranges. Clamped silently.
// NOTE (Issue #18): with the admissible model, alpha now scales a PENALTY on
// straight edges (factor up to 1 + alpha * kCurvyDetourCap) instead of a
// discount on curvy ones — the alpha scale changed meaning. The 0.6 default
// is carried over from v1 and needs recalibration on the Stage-4 harness.
constexpr float kDefaultCurvyAlpha = 0.6f;
constexpr ranged_default_t<float> kCurvyAlphaRange{0.0f, kDefaultCurvyAlpha, 0.95f};
constexpr float kDefaultUseScenicTolls = 0.5f;
constexpr ranged_default_t<float> kUseScenicTollsRange{0.2f, kDefaultUseScenicTolls, 0.7f};

// Profile-rework D1: motorcycle_curvy now READS use_highways / use_trails from
// the request (they were hardcoded). These curvy-specific defaults keep a bare
// request at today's strong-avoid character (uh=0.1, ut=0.0) rather than
// falling to the stock use_highways default of 0.5.
constexpr float kDefaultCurvyUseHighways = 0.1f; // historical motorcycle_curvy value
constexpr ranged_default_t<float> kCurvyUseHighwaysRange{0.0f, kDefaultCurvyUseHighways, 1.0f};
constexpr float kDefaultCurvyUseTrails = 0.0f;
constexpr ranged_default_t<float> kCurvyUseTrailsRange{0.0f, kDefaultCurvyUseTrails, 1.0f};
// Patch 0023: motorcycle_curvy now READS use_tolls from the request. The
// default is the 0.2 the parser used to hardcode, NOT the stock 0.5, so a
// bare request costs tolls exactly as it did before this patch.
constexpr float kDefaultCurvyUseTolls = 0.2f;
constexpr ranged_default_t<float> kCurvyUseTollsRange{0.0f, kDefaultCurvyUseTolls, 1.0f};
// D2 (profile character): use_small_roads scales the four small-road class-mult
// rows toward 1.0. Default 0.0 keeps today's table for every profile that does
// not send it.
constexpr float kDefaultUseSmallRoads = 0.0f;
constexpr ranged_default_t<float> kCurvyUseSmallRoadsRange{0.0f, kDefaultUseSmallRoads, 1.0f};

constexpr float kHighwayFactor[] = {
    1.0f, // Motorway
    0.5f, // Trunk
    0.0f, // Primary
    0.0f, // Secondary
    0.0f, // Tertiary
    0.0f, // Unclassified
    0.0f, // Residential
    0.0f  // Service, other
};

constexpr float kMaxTrailBiasFactor = 8.0f;

constexpr float kSurfaceFactor[] = {
    0.0f, // kPavedSmooth
    0.0f, // kPaved
    0.0f, // kPaveRough
    0.1f, // kCompacted
    0.2f, // kDirt
    0.5f, // kGravel
    1.0f  // kPath
};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.disable_rail_ferry_ = true;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

// Preferred-trail option (patch 0019), parsed for motorcycle + motorcycle_curvy
// only. `/preferred_edges` = JSON array of uint64 directed-edge GraphIds;
// `/preferred_factor` = >= 1.0 penalty on non-member edges (floored at 1.0 so
// the cost model stays admissible). Both absent => byte-identical legacy wire.
void ParsePreferredEdges(const rapidjson::Value& json, Costing::Options* co) {
  if (auto edges = rapidjson::get_child_optional(json, "/preferred_edges");
      edges && edges->IsArray()) {
    for (const auto& e : edges->GetArray()) {
      if (e.IsUint64()) {
        co->add_preferred_edges(e.GetUint64());
      }
    }
  }
  if (auto f = rapidjson::get_optional<float>(json, "/preferred_factor"); f) {
    co->set_preferred_factor(std::max(1.0f, *f));
  }
}

// The four preference-driven factors of MotorcycleCost, as pure functions of
// their knob so the profile set and the in-city set (patch 0025) are built
// by the SAME formulas. Bodies are the stock constructor's, unchanged.

// Factor for highway use - use a non-linear factor with values at 0.5 being neutral (factor
// of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
// highways) while values between 0.5 to 0 slowly increase to a maximum of kMaxHighwayBiasFactor
// to avoid/penalize highways.
float HighwayFactor(float use_highways) {
  if (use_highways >= 0.5f) {
    float f = (0.5f - use_highways);
    return f * f * f;
  }
  float f = 1.0f - (use_highways * 2.0f);
  return kMaxHighwayBiasFactor * (f * f);
}

// Toll factor of 0 would indicate no adjustment to weighting for toll roads.
// use_tolls = 1 would reduce weighting slightly (a negative delta) while
// use_tolls = 0 would penalize (positive delta to weighting factor).
float TollFactor(float use_tolls) {
  return use_tolls < 0.5f ? (2.0f - 4 * use_tolls) : // ranges from 2 to 0
             (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.015
}

// Factor for trail use - use a non-linear factor with values at 0.5 being neutral (factor
// of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
// trails) while values between 0.5 to 0 slowly increase to a maximum of the surfact_factor_
// to avoid/penalize trails.
float TrailSurfaceFactor(float use_trails) {
  if (use_trails >= 0.5f) {
    float f = (0.5f - use_trails);
    return f * f * f;
  }
  float f = 1.0f - use_trails * 2.0f;
  return static_cast<uint32_t>(kMaxTrailBiasFactor * (f * f));
}

// The ferry weighting of DynamicCost's constructor (dynamiccost.h), for the
// in-city use_ferry: edge factor + entry cost.
float FerryFactor(float use_ferry) {
  return use_ferry < 0.5f ? 10.0f - use_ferry * 18.0f : 1.5f - use_ferry;
}
Cost FerryTransitionCost(float use_ferry, float ferry_cost) {
  const float penalty =
      use_ferry < 0.5f ? static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - use_ferry * 2.0f)) : 0.0f;
  return {ferry_cost + penalty, ferry_cost};
}

// City settings (patch 0025, Vamoto #193), parsed for motorcycle and
// motorcycle_curvy. PRESENCE matters (any city_use_* present activates the
// in-city set), so get_optional + clamp -- never JSON_PBF_RANGED_DEFAULT,
// which always sets. Absent keys leave the pbf alone, so a second ParseApi
// pass over a pre-filled request (the mobile shim's trails branch) keeps
// the first pass's values.
void ParseCityOptions(const rapidjson::Value& json, Costing::Options* co) {
  if (auto v = rapidjson::get_optional<bool>(json, "/city_aversion")) {
    co->set_city_aversion(*v);
  }
  if (auto v = rapidjson::get_optional<bool>(json, "/city_fastest")) {
    co->set_city_fastest(*v);
  }
  if (auto v = rapidjson::get_optional<float>(json, "/city_use_highways")) {
    co->set_city_use_highways(std::clamp(*v, 0.0f, 1.0f));
  }
  if (auto v = rapidjson::get_optional<float>(json, "/city_use_tolls")) {
    co->set_city_use_tolls(std::clamp(*v, 0.0f, 1.0f));
  }
  if (auto v = rapidjson::get_optional<float>(json, "/city_use_ferry")) {
    co->set_city_use_ferry(std::clamp(*v, 0.0f, 1.0f));
  }
  if (auto v = rapidjson::get_optional<float>(json, "/city_use_trails")) {
    co->set_city_use_trails(std::clamp(*v, 0.0f, 1.0f));
  }
  if (auto v = rapidjson::get_optional<float>(json, "/city_use_scenic_tolls")) {
    co->set_city_use_scenic_tolls(std::clamp(*v, 0.2f, 0.7f));
  }
}

} // namespace

/**
 * Derived class providing dynamic edge costing for "direct" auto routes. This
 * is a route that is generally shortest time but uses route hierarchies that
 * can result in slightly longer routes that avoid shortcuts on residential
 * roads.
 */
class MotorcycleCost : public DynamicCost {
public:
  /**
   * Construct motorcycle costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  MotorcycleCost(const Costing& costing_options);

  virtual ~MotorcycleCost();

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const override {
    return true;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  is_dest                     Is a directed edge the destination?
   * @param  pred                        Predecessor edge information.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the directed edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx,
                       uint8_t& destonly_access_restr_mask) const override;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  pred                        Predecessor edge information.
   * @param  opp_edge                    Pointer to the opposing directed edge.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the opposing edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx,
                              uint8_t& destonly_access_restr_mask) const override;

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("MotorcycleCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphId& edgeid,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge          Directed edge (the to edge)
   * @param  node          Node (intersection) where transition occurs.
   * @param  pred          Predecessor edge information.
   * @param  tile          Pointer to the graph tile containing the to edge.
   * @param  reader_getter Functor that facilitates access to a limited version of the graph reader
   * @return Returns the cost and time (seconds)
   */
  virtual Cost
  TransitionCost(const baldr::DirectedEdge* edge,
                 const baldr::NodeInfo* node,
                 const EdgeLabel& pred,
                 const graph_tile_ptr& tile,
                 const std::function<LimitedGraphReader()>& reader_getter) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx                Directed edge local index
   * @param  node               Node (intersection) where transition occurs.
   * @param  pred               the opposing current edge in the reverse tree.
   * @param  edge               the opposing predecessor in the reverse tree
   * @param  tile               Graphtile that contains the node and the opp_edge
   * @param  edge_id            Graph ID of opp_pred_edge to get its tile if needed
   * @param  reader_getter      Functor that facilitates access to a limited version of the graph
   * reader
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn      Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge,
                                     const graph_tile_ptr& tile,
                                     const GraphId& pred_id,
                                     const std::function<LimitedGraphReader()>& reader_getter,
                                     const bool has_measured_speed,
                                     const InternalTurn /*internal_turn*/) const override;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override {
    return kSpeedFactor[top_speed_] * min_linear_cost_factor_;
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override {
    return static_cast<uint8_t>(VehicleType::kMotorcycle);
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by automobile.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (allow_closures || !tile->IsClosed(edge));
  }
  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
public:
  VehicleType type_;     // Vehicle type: car (default), motorcycle, etc
  float toll_factor_;    // Factor applied when road has a toll
  float surface_factor_; // How much the surface factors are applied when using trails
  float highway_factor_; // Factor applied when road is a motorway or trunk

  // City settings (patch 0025, Vamoto #193). An edge is "in a city" when
  // in_city(edge->density()). city_set_active_ swaps the four factors above
  // (and the ferry weighting) for the city_* ones on in-city edges.
  bool city_aversion_;
  bool city_fastest_;
  bool city_set_active_;
  float city_use_highways_;
  float city_use_trails_;
  float city_toll_factor_;
  float city_surface_factor_;
  float city_highway_factor_;
  float city_ferry_factor_;
  Cost city_ferry_transition_cost_;

  // Patch 0026: the in-city ferry entry swap of TransitionCost/Reverse. Its
  // cost part follows base_transition_cost's `shortest` rule (a shortest
  // search ignores penalties), so the swap never leaves a residue there.
  Cost CityFerryEntrySwap(const baldr::DirectedEdge* edge, Use pred_use) const {
    if (!city_set_active_ || edge->use() != Use::kFerry || pred_use == Use::kFerry ||
        !in_city(edge->density())) {
      return {0.0f, 0.0f};
    }
    return {(city_ferry_transition_cost_.cost - ferry_transition_cost_.cost) * !shortest_,
            city_ferry_transition_cost_.secs - ferry_transition_cost_.secs};
  }
};

// Constructor
MotorcycleCost::MotorcycleCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kMotorcycleAccess) {

  const auto& costing_options = costing.options();

  // Vehicle type is motorcycle
  type_ = VehicleType::kMotorcycle;

  // Get the base costs
  get_base_costs(costing);

  // Preference to use highways, tolls and trails (each a value from 0 to 1).
  highway_factor_ = HighwayFactor(costing_options.use_highways());
  toll_factor_ = TollFactor(costing_options.use_tolls());
  surface_factor_ = TrailSurfaceFactor(costing_options.use_trails());

  // City settings (patch 0025). A missing member of an active in-city set
  // falls back to the profile's own value.
  city_aversion_ = costing_options.city_aversion();
  city_fastest_ = costing_options.city_fastest();
  city_set_active_ = costing_options.has_city_use_highways_case() ||
                     costing_options.has_city_use_tolls_case() ||
                     costing_options.has_city_use_ferry_case() ||
                     costing_options.has_city_use_trails_case();
  city_use_highways_ = costing_options.has_city_use_highways_case()
                           ? costing_options.city_use_highways()
                           : costing_options.use_highways();
  city_use_trails_ = costing_options.has_city_use_trails_case() ? costing_options.city_use_trails()
                                                                : costing_options.use_trails();
  const float city_use_tolls = costing_options.has_city_use_tolls_case()
                                   ? costing_options.city_use_tolls()
                                   : costing_options.use_tolls();
  const float city_use_ferry = costing_options.has_city_use_ferry_case()
                                   ? costing_options.city_use_ferry()
                                   : costing_options.use_ferry();
  city_highway_factor_ = HighwayFactor(city_use_highways_);
  city_toll_factor_ = TollFactor(city_use_tolls);
  city_surface_factor_ = TrailSurfaceFactor(city_use_trails_);
  city_ferry_factor_ = FerryFactor(city_use_ferry);
  city_ferry_transition_cost_ = FerryTransitionCost(city_use_ferry, costing_options.ferry_cost());
}

// Destructor
MotorcycleCost::~MotorcycleCost() {
}

// Check if access is allowed on the specified edge.
bool MotorcycleCost::Allowed(const baldr::DirectedEdge* edge,
                             const bool is_dest,
                             const EdgeLabel& pred,
                             const graph_tile_ptr& tile,
                             const baldr::GraphId& edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index,
                             uint8_t& restriction_idx,
                             uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !ignore_turn_restrictions_) ||
      (edge->surface() > kMinimumMotorcycleSurface) || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) || CheckExclusions<true>(edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx, destonly_access_restr_mask);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool MotorcycleCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                    const EdgeLabel& pred,
                                    const baldr::DirectedEdge* opp_edge,
                                    const graph_tile_ptr& tile,
                                    const baldr::GraphId& opp_edgeid,
                                    const uint64_t current_time,
                                    const uint32_t tz_index,
                                    uint8_t& restriction_idx,
                                    uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(opp_edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) && !ignore_turn_restrictions_) ||
      (opp_edge->surface() > kMinimumMotorcycleSurface) || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      CheckExclusions<false>(opp_edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, opp_edge, false, tile, opp_edgeid,
                                           current_time, tz_index, restriction_idx,
                                           destonly_access_restr_mask);
}

Cost MotorcycleCost::EdgeCost(const baldr::DirectedEdge* edge,
                              const baldr::GraphId& edgeid,
                              const graph_tile_ptr& tile,
                              const baldr::TimeInfo& time_info,
                              uint8_t& flow_sources) const {
  auto edge_speed = fixed_speed_ == baldr::kDisableFixedSpeed
                        ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, false,
                                         &flow_sources, time_info.seconds_from_now)
                        : fixed_speed_;

  auto final_speed = std::min(edge_speed, top_speed_);

  float sec = (edge->length() * kSpeedFactor[final_speed]);

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  // Patch 0025: in-city edges cost with the in-city option set when one is
  // active (setting 3 / the fastest preset of setting 2).
  const bool city_edge = in_city(edge->density());
  const bool city_set = city_set_active_ && city_edge;

  // Special case for travel on a ferry
  if (edge->use() == Use::kFerry) {
    // Use the edge speed (should be the speed of the ferry)
    return {sec * (city_set ? city_ferry_factor_ : ferry_factor_), sec};
  }

  // ETA credibility: recompute elapsed time with a speed floor so mandatory
  // low-class connectors (tracks/paths/driveways at Valhalla's walking-pace
  // default speed) don't blow up the ETA. Route choice is unaffected — the
  // cost factors below still steer onto real roads. (See kMotorcycleMinEtaSpeed.)
  const uint32_t eta_speed = std::max<uint32_t>(final_speed, kMotorcycleMinEtaSpeed);
  sec = (edge->length() * kSpeedFactor[eta_speed]);

  float factor = kDensityFactor[edge->density()] +
                 (city_set ? city_highway_factor_ : highway_factor_) *
                     kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
                 (city_set ? city_surface_factor_ : surface_factor_) *
                     kSurfaceFactor[static_cast<uint32_t>(edge->surface())];
  factor += SpeedPenalty(edge, tile, time_info, flow_sources, edge_speed);
  if (edge->toll()) {
    factor += city_set ? city_toll_factor_ : toll_factor_;
  }

  if (edge->use() == Use::kTrack) {
    factor *= track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= service_factor_;
  }
  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }

  factor *= EdgeFactor(edgeid);

  // Patch 0025 setting 1, discourage city driving: graded, >= 1.0, motorway
  // and trunk exempt. Here in the base EdgeCost so motorcycle_curvy inherits
  // it through base.cost, the city-fastest shortcut included.
  if (city_aversion_ && city_edge) {
    factor *= city_aversion_factor(edge->density(), edge->classification(),
                                   edge->use() == Use::kRamp);
  }

  // Preferred trails (patch 0019): edges outside the per-request preferred set
  // pay a flat >= 1.0 penalty, biasing the router onto the trail network without
  // ever discounting below base cost. No-op when the set is empty or the factor
  // is <= 1.0 (strength Off). Applied ONLY here in the base motorcycle EdgeCost
  // so motorcycle_curvy inherits it through base.cost -- do NOT repeat the
  // multiply in MotorcycleCurvyCost::EdgeCost (that would double-apply it).
  factor *= PreferredEdgeFactor(edgeid);

  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost MotorcycleCost::TransitionCost(
    const baldr::DirectedEdge* edge,
    const baldr::NodeInfo* node,
    const EdgeLabel& pred,
    const graph_tile_ptr& /*tile*/,
    const std::function<LimitedGraphReader()>& /*reader_getter*/) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);
  // Patch 0025: entering an in-city ferry pays the in-city use_ferry entry.
  c += CityFerryEntrySwap(edge, pred.use());
  c.secs += OSRMCarTurnDuration(edge, node, idx);

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred.use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred.use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;
    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;

    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false, InternalTurn::kNoTurn,
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!pred.has_measured_speed()) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost MotorcycleCost::TransitionCostReverse(
    const uint32_t idx,
    const baldr::NodeInfo* node,
    const baldr::DirectedEdge* pred,
    const baldr::DirectedEdge* edge,
    const graph_tile_ptr& /*tile*/,
    const GraphId& /*pred_id*/,
    const std::function<LimitedGraphReader()>& /*reader_getter*/,
    const bool has_measured_speed,
    const InternalTurn /*internal_turn*/) const {

  // Motorcycles should be able to make uturns on short internal edges; therefore, InternalTurn
  // is ignored for now.
  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);
  // Patch 0025: entering an in-city ferry pays the in-city use_ferry entry.
  c += CityFerryEntrySwap(edge, pred->use());
  c.secs += OSRMCarTurnDuration(edge, node, pred->opp_local_idx());

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred->use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred->use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;

    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;

    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false, InternalTurn::kNoTurn,
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

void ParseMotorcycleCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                Costing* c,
                                google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  c->set_type(Costing::motorcycle);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseHighwaysRange, json, "/use_highways", use_highways, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseTollsRange, json, "/use_tolls", use_tolls, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseTrailsRange, json, "/use_trails", use_trails, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kMotorcycleSpeedRange, json, "/top_speed", top_speed, warnings);
  ParsePreferredEdges(json, co);
  ParseCityOptions(json, co);
}

cost_ptr_t CreateMotorcycleCost(const Costing& costing_options) {
  return std::make_shared<MotorcycleCost>(costing_options);
}

/**
 * MotorcycleCurvyCost — curvy-routing variant of MotorcycleCost
 * (better_mc_routing v1). Issue 06 layers the per-edge curvy bonus on
 * top of MotorcycleCost's existing factor; Issues 07-08 layer class
 * multipliers and the scenic-toll heuristic on top of THAT.
 */
class MotorcycleCurvyCost : public MotorcycleCost {
public:
  MotorcycleCurvyCost(const Costing& costing_options) : MotorcycleCost(costing_options) {
    // ParseMotorcycleCurvyCostOptions already clamped these via ranged_default_t.
    curvy_alpha_ = costing_options.options().curvy_alpha();
    use_scenic_tolls_ = costing_options.options().use_scenic_tolls();
    // D1/D2: scale motorway/trunk (use_highways), track (use_trails) and the
    // four small-road rows (use_small_roads) ONCE here; EdgeCost reads
    // class_mult_ + use_trails_ per edge (no per-edge table recompute).
    const float uh = costing_options.options().use_highways();
    const float ut = costing_options.options().use_trails();
    const float usr = costing_options.options().use_small_roads();
    use_trails_ = ut;
    class_mult_ = scaled_class_multipliers(uh, ut, usr);
    // Patch 0025: the in-city table, from the in-city use_highways /
    // use_trails (the profile's when the request sent none) and the
    // profile's use_small_roads.
    city_class_mult_ = scaled_class_multipliers(city_use_highways_, city_use_trails_, usr);
    city_use_scenic_tolls_ = costing_options.options().has_city_use_scenic_tolls_case()
                                 ? costing_options.options().city_use_scenic_tolls()
                                 : use_scenic_tolls_;
    // Patch 0026 (F1): the in-city fastest cost sits on the curvy scale.
    city_fastest_scale_ = city_fastest_scale(curvy_alpha_, class_mult_);
  }

  Cost EdgeCost(const baldr::DirectedEdge* edge,
                const baldr::GraphId& edgeid,
                const graph_tile_ptr& tile,
                const baldr::TimeInfo& time_info,
                uint8_t& flow_sources) const override {
    Cost base = MotorcycleCost::EdgeCost(edge, edgeid, tile, time_info, flow_sources);
    // Patch 0025 setting 2: inside a city the route costs like the fastest
    // preset -- the base motorcycle cost (already on the in-city factor set,
    // the discourage-city factor included), no curvy multipliers. Patch
    // 0026 (F1): times ONE per-request constant, the curvy costing's charge
    // for a straight main road, so a city is never cheaper than the
    // countryside around it (see city_fastest_scale).
    const bool city_edge = in_city(edge->density());
    if (city_fastest_ && city_edge) {
      return Cost(base.cost * city_fastest_scale_, base.secs);
    }
    const bool city_set = city_set_active_ && city_edge;
    // Issue #20 — hot path: read the sinuosity byte from the DirectedEdgeExt
    // record (plain pointer arithmetic) instead of tile->edgeinfo(edge),
    // which re-parses the tagged values (hash map build) per EdgeCost call.
    // Tiles built without ext data fall back to 0 (= no curvy preference).
    // An invalid edgeid (PartialEdgeCost passes GraphId(kInvalidGraphId)
    // during Loki location snapping, deliberately, to skip the whole-edge
    // factor) must also fall back to 0 — ext_directededge() would otherwise
    // throw on the out-of-range sentinel index. Snapping wants the neutral
    // (straight) cost anyway.
    const uint8_t sin_byte = (edgeid.is_valid() && tile->header()->has_ext_directededge())
                                 ? tile->ext_directededge(edgeid)->sinuosity()
                                 : 0;
    // v3: don't let the curve preference pull a SURFACE-AVERSE profile onto
    // curvy GRAVEL. On unpaved edges (baldr::Surface >= kCompacted = 3) for a
    // profile that avoids bad surfaces (surface_factor_ high == use_trails low,
    // e.g. sport_touring), withhold the curve discount — score the edge as if
    // straight (byte 0 -> max straightness penalty) so curvy gravel is never
    // cheaper than paved. Adventure (surface_factor_ ~0, use_trails high) keeps
    // the curve preference on gravel.
    const bool unpaved = static_cast<uint8_t>(edge->surface()) >= 3;
    // Patch 0024 (#192): a ramp is not a curve worth riding -- its tight
    // radius earns no curve reward.
    const bool ramp = edge->use() == Use::kRamp;
    const float surface_factor = city_set ? city_surface_factor_ : surface_factor_;
    const uint8_t curve_byte = ((unpaved && surface_factor > 1.0f) || ramp) ? 0 : sin_byte;
    // Issue #18 — admissible cost model: every factor below is >= 1.0, so
    // EdgeCost(motorcycle_curvy) >= EdgeCost(motorcycle) on every edge and
    // the A* heuristic calibrated against base costs stays admissible.
    const float sp = straightness_penalty(curve_byte, curvy_alpha_);
    const float cm = class_multiplier(edge->classification(), edge->use(),
                                      city_set ? city_class_mult_ : class_mult_);
    const float tm = toll_multiplier(edge->toll(), edge->classification(),
                                     city_set ? city_use_scenic_tolls_ : use_scenic_tolls_);
    // D1 rev.2: speed-equalized paved penalty (adventure detours onto gravel).
    // Uses the tile's assigned speed (deterministic, traffic-free).
    const float pm = paved_multiplier(edge->surface(), city_set ? city_use_trails_ : use_trails_,
                                      static_cast<float>(edge->speed()));
    return Cost(base.cost * sp * cm * tm * pm, base.secs);
  }

  // Patch 0024 (#192): the highway-ramp transition penalty, on BOTH searches
  // (bidirectional A* calls the forward and the reverse variant).
  Cost TransitionCost(const baldr::DirectedEdge* edge,
                      const baldr::NodeInfo* node,
                      const EdgeLabel& pred,
                      const graph_tile_ptr& tile,
                      const std::function<LimitedGraphReader()>& reader_getter) const override {
    Cost c = MotorcycleCost::TransitionCost(edge, node, pred, tile, reader_getter);
    // Patch 0026 (F1): the ramp penalty applies in a city too, setting 2
    // included -- waiving it made in-city interchanges a cheap detour.
    if (highway_ramp_transition(pred.classification(), pred.use() == Use::kRamp,
                                edge->classification(), edge->use() == Use::kRamp)) {
      c.cost += kCurvyHighwayRampPenalty;
    }
    return c;
  }

  Cost TransitionCostReverse(const uint32_t idx,
                             const baldr::NodeInfo* node,
                             const baldr::DirectedEdge* pred,
                             const baldr::DirectedEdge* edge,
                             const graph_tile_ptr& tile,
                             const GraphId& pred_id,
                             const std::function<LimitedGraphReader()>& reader_getter,
                             const bool has_measured_speed,
                             const InternalTurn internal_turn) const override {
    Cost c = MotorcycleCost::TransitionCostReverse(idx, node, pred, edge, tile, pred_id,
                                                   reader_getter, has_measured_speed,
                                                   internal_turn);
    if (highway_ramp_transition(pred->classification(), pred->use() == Use::kRamp,
                                edge->classification(), edge->use() == Use::kRamp)) {
      c.cost += kCurvyHighwayRampPenalty;
    }
    return c;
  }

protected:
  float curvy_alpha_;
  float use_scenic_tolls_;
  float use_trails_;             // D1 rev.2: scalar for the per-edge paved_multiplier
  ClassMultipliers class_mult_; // compile-time defaults, see scenic_cost_helpers.h
  // Patch 0025: the in-city counterparts (used only when city_set_active_).
  ClassMultipliers city_class_mult_;
  float city_use_scenic_tolls_;
  float city_fastest_scale_; // patch 0026: >= 1.0, see city_fastest_scale
};

void ParseMotorcycleCurvyCostOptions(const rapidjson::Document& doc,
                                     const std::string& costing_options_key,
                                     Costing* c,
                                     google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  c->set_type(Costing::motorcycle_curvy);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  // Profile-rework D1: use_highways and use_trails are now LIVE knobs for
  // motorcycle_curvy — they were previously hardcoded, silently discarding
  // whatever the request sent. Absent, they fall to curvy-specific defaults
  // (uh=0.1, ut=0.0) that preserve legacy no-options routing. Patch 0023 did
  // the same for use_tolls (default 0.2); top_speed stays hardcoded.
  //   use_highways (default 0.1)  scales motorway/trunk rows + base highway_factor_
  //   use_tolls (default 0.2)     feeds MotorcycleCost's toll_factor_ (:378-380),
  //                               added per tolled edge in EdgeCost (:490). Our
  //                               toll_multiplier adds a 1.6x on top for road
  //                               tolls; scenic tolls scale separately through
  //                               use_scenic_tolls, which this patch leaves alone.
  //   use_trails (default 0.0)    scales the track row + base surface_factor_
  //   top_speed = 120 km/h        discourage routing into 140+ km/h motorway
  //                               edges via ETA component
  ParseBaseCostOptions(json, c, kBaseCostOptsConfig, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kCurvyUseHighwaysRange, json, "/use_highways", use_highways, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kCurvyUseTollsRange, json, "/use_tolls", use_tolls, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kCurvyUseTrailsRange, json, "/use_trails", use_trails, warnings);
  co->set_top_speed(120);    // unchanged
  JSON_PBF_RANGED_DEFAULT(co, kCurvyAlphaRange, json, "/curvy_alpha", curvy_alpha, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseScenicTollsRange, json, "/use_scenic_tolls", use_scenic_tolls,
                          warnings);
  // D2 (profile character): scale the four small-road rows toward 1.0.
  JSON_PBF_RANGED_DEFAULT(co, kCurvyUseSmallRoadsRange, json, "/use_small_roads", use_small_roads,
                          warnings);
  ParsePreferredEdges(json, co);
  ParseCityOptions(json, co);
}

cost_ptr_t CreateMotorcycleCurvyCost(const Costing& costing_options) {
  return std::make_shared<MotorcycleCurvyCost>(costing_options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestMotorcycleCost : public MotorcycleCost {
public:
  TestMotorcycleCost(const Costing& costing_options) : MotorcycleCost(costing_options){};

  using MotorcycleCost::alley_penalty_;
  using MotorcycleCost::country_crossing_cost_;
  using MotorcycleCost::destination_only_penalty_;
  using MotorcycleCost::ferry_transition_cost_;
  using MotorcycleCost::gate_cost_;
  using MotorcycleCost::maneuver_penalty_;
  using MotorcycleCost::service_factor_;
  using MotorcycleCost::service_penalty_;
  using MotorcycleCost::toll_booth_cost_;
};

TestMotorcycleCost* make_motorcyclecost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "motorcycle", "costing_options":{"motorcycle":{")" << property << R"(":)"
     << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestMotorcycleCost(request.options().costings().find(Costing::motorcycle)->second);
}

template <typename T>
std::uniform_real_distribution<T>* make_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

TEST(MotorcycleCost, testMotorcycleCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestMotorcycleCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("maneuver_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("alley_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("destination_only_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("gate_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("gate_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // toll_booth_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("toll_booth_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.secs,
                test::IsBetween(defaults.toll_booth_cost_.min, defaults.toll_booth_cost_.max));
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("toll_booth_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.cost,
                test::IsBetween(defaults.toll_booth_penalty_.min,
                                defaults.toll_booth_penalty_.max + defaults.toll_booth_cost_.def));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("country_crossing_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorcyclecost_from_json("country_crossing_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("ferry_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // service_penalty_
  distributor.reset(make_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("service_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // service_factor_
  distributor.reset(make_distributor_from_range(defaults.service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorcyclecost_from_json("service_factor", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_factor_,
                test::IsBetween(defaults.service_factor_.min, defaults.service_factor_.max));
  }

  /*
   // use_ferry
   distributor.reset(make_distributor_from_range(defaults.use_ferry_));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_motorcyclecost_from_json("use_ferry", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_ferry , test::IsBetween(defaults.use_ferry_.min,
defaults.use_ferry_.max));
   }

    // use_highways
    distributor.reset(make_distributor_from_range(kUseHighwaysRange));
    for (unsigned i = 0; i < testIterations; ++i) {
      ctorTester.reset(make_motorcyclecost_from_json("use_highways", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_highways , test::IsBetween(kUseHighwaysRange.min, kUseHighwaysRange.max));
    }

     // use_trails
     distributor.reset(make_distributor_from_range(kUseTrailsRange));
     for (unsigned i = 0; i < testIterations; ++i) {
       ctorTester.reset(make_motorcyclecost_from_json("use_trails", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_trails , test::IsBetween(kUseTrailsRange.min, kUseTrailsRange.max));
     }

   // use_tolls
   distributor.reset(make_distributor_from_range(kUseTollsRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_motorcyclecost_from_json("use_tolls", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_tolls , test::IsBetween(kUseTollsRange.min, kUseTollsRange.max));
   }
   **/
}

TEST(MotorcycleCost, etaSpeedFloor) {
  // A sub-floor speed must yield an elapsed time no slower than the floor, and a
  // supra-floor speed must be unchanged. kSpeedFactor is seconds-per-metre at a
  // given km/h, so a higher speed => smaller factor.
  const uint32_t floor = kMotorcycleMinEtaSpeed; // 25
  EXPECT_EQ(std::max<uint32_t>(5u, floor), floor);
  EXPECT_EQ(std::max<uint32_t>(80u, floor), 80u);
  // time for a 1 km track edge at the raw 5 km/h default vs. the floored speed:
  const float raw = 1000.0f * kSpeedFactor[5];
  const float floored = 1000.0f * kSpeedFactor[std::max<uint32_t>(5u, floor)];
  EXPECT_LT(floored, raw); // floor makes it faster/credible
  EXPECT_NEAR(floored, 1000.0f * kSpeedFactor[25], 1e-4);
}

// ---- preferred trails (patch 0019) ----

// Subclasses that expose the protected preferred-trail members + helper so the
// unit tests can assert the parse and the multiplier without a tile.
class TestMotorcyclePreferred : public MotorcycleCost {
public:
  TestMotorcyclePreferred(const Costing& c) : MotorcycleCost(c) {}
  using DynamicCost::PreferredEdgeFactor;
  using DynamicCost::preferred_edges_;
  using DynamicCost::preferred_factor_;
};

class TestMotorcycleCurvyPreferred : public MotorcycleCurvyCost {
public:
  TestMotorcycleCurvyPreferred(const Costing& c) : MotorcycleCurvyCost(c) {}
  using DynamicCost::PreferredEdgeFactor;
  using DynamicCost::preferred_edges_;
  using DynamicCost::preferred_factor_;
};

Costing parse_preferred_costing(const std::string& costing, const std::string& body) {
  std::stringstream ss;
  ss << R"({"costing":")" << costing << R"(","costing_options":{")" << costing << R"(":)" << body
     << "}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  const auto type = costing == "motorcycle" ? Costing::motorcycle : Costing::motorcycle_curvy;
  return request.options().costings().find(type)->second;
}

TEST(MotorcycleCost, PreferredEdgesParsedAndApplied) {
  TestMotorcyclePreferred cost(
      parse_preferred_costing("motorcycle",
                              R"({"preferred_edges":[12345,67890],"preferred_factor":3.0})"));
  EXPECT_EQ(cost.preferred_edges_.size(), 2u);
  EXPECT_FLOAT_EQ(cost.preferred_factor_, 3.0f);
  // A registered edge pays base cost; an unregistered one pays the factor.
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(12345))), 1.0f);
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(999999))), 3.0f);
}

TEST(MotorcycleCurvyCost, PreferredEdgesInherited) {
  // motorcycle_curvy carries the preferred set through its MotorcycleCost base
  // (MotorcycleCurvyCost::EdgeCost multiplies base.cost, which already includes
  // PreferredEdgeFactor). The gurka curvy path has a pre-existing crash, so the
  // inheritance is proven here at the costing level instead.
  TestMotorcycleCurvyPreferred cost(
      parse_preferred_costing("motorcycle_curvy",
                              R"({"preferred_edges":[12345,67890],"preferred_factor":3.0})"));
  EXPECT_EQ(cost.preferred_edges_.size(), 2u);
  EXPECT_FLOAT_EQ(cost.preferred_factor_, 3.0f);
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(12345))), 1.0f);
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(999999))), 3.0f);
}

TEST(MotorcycleCost, PreferredFactorFlooredAtOne) {
  // A factor below 1.0 is floored to 1.0 at parse time, so the bias can only
  // ever raise cost (admissibility) -- and a floored factor is a no-op.
  TestMotorcyclePreferred cost(
      parse_preferred_costing("motorcycle",
                              R"({"preferred_edges":[12345],"preferred_factor":0.5})"));
  EXPECT_FLOAT_EQ(cost.preferred_factor_, 1.0f);
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(999999))), 1.0f);
}

TEST(MotorcycleCost, PreferredEdgesAbsentIsNoOp) {
  // No preferred fields -> empty set, factor 1.0, no-op multiplier.
  TestMotorcyclePreferred cost(parse_preferred_costing("motorcycle", R"({})"));
  EXPECT_TRUE(cost.preferred_edges_.empty());
  EXPECT_FLOAT_EQ(cost.preferred_factor_, 1.0f);
  EXPECT_FLOAT_EQ(cost.PreferredEdgeFactor(GraphId(static_cast<uint64_t>(12345))), 1.0f);
}

TEST(MotorcycleCurvyCost, UseTollsIsParsedNotHardcoded) {
  // Patch 0023. Before it, ParseMotorcycleCurvyCostOptions called
  // set_use_tolls(0.2f) unconditionally and discarded the request's value,
  // so "avoid tolls" could not reach the curvy costing at all.
  // parse_preferred_costing (patch 0019) is the generic parse harness here.
  EXPECT_FLOAT_EQ(parse_preferred_costing("motorcycle_curvy", R"({})").options().use_tolls(),
                  kDefaultCurvyUseTolls);
  EXPECT_FLOAT_EQ(
      parse_preferred_costing("motorcycle_curvy", R"({"use_tolls":0.0})").options().use_tolls(),
      0.0f);
  EXPECT_FLOAT_EQ(
      parse_preferred_costing("motorcycle_curvy", R"({"use_tolls":1.0})").options().use_tolls(),
      1.0f);
}

TEST(MotorcycleCurvyCost, UseTollsAbsentKeepsTheLegacyValue) {
  // The whole point of the curvy-specific default: a request that says
  // nothing about tolls must cost EXACTLY as it did before patch 0023, so
  // the shared parity vectors do not move. 0.2, not the stock 0.5.
  EXPECT_FLOAT_EQ(kDefaultCurvyUseTolls, 0.2f);
  EXPECT_NE(kDefaultCurvyUseTolls, kDefaultUseTolls);
  MotorcycleCurvyCost absent(parse_preferred_costing("motorcycle_curvy", R"({})"));
  MotorcycleCurvyCost explicit_legacy(
      parse_preferred_costing("motorcycle_curvy", R"({"use_tolls":0.2})"));
  EXPECT_FLOAT_EQ(absent.toll_factor_, explicit_legacy.toll_factor_);
}

TEST(MotorcycleCurvyCost, UseTollsOutOfRangeSnapsToTheDefault) {
  // ranged_default_t SNAPS, it does not clamp -- the same trap that made
  // use_scenic_tolls = 0.0 land on 0.5. Pinned here so nobody "fixes" the
  // range by widening it.
  for (const auto* body : {R"({"use_tolls":5.0})", R"({"use_tolls":-1.0})"}) {
    EXPECT_FLOAT_EQ(parse_preferred_costing("motorcycle_curvy", body).options().use_tolls(),
                    kDefaultCurvyUseTolls);
  }
}

TEST(MotorcycleCurvyCost, UseTollsReachesTheTollFactor) {
  // Parsed is not the same as consumed. MotorcycleCost's ctor turns the
  // knob into toll_factor_ (:378-380) and EdgeCost adds it on a tolled edge
  // (:490); MotorcycleCurvyCost inherits both, so avoiding tolls really
  // does get more expensive on the curvy costing.
  MotorcycleCurvyCost avoided(parse_preferred_costing("motorcycle_curvy", R"({"use_tolls":0.0})"));
  MotorcycleCurvyCost legacy(parse_preferred_costing("motorcycle_curvy", R"({})"));
  MotorcycleCurvyCost welcomed(
      parse_preferred_costing("motorcycle_curvy", R"({"use_tolls":1.0})"));
  EXPECT_GT(avoided.toll_factor_, legacy.toll_factor_);
  EXPECT_GT(legacy.toll_factor_, welcomed.toll_factor_);
}

// ---- city settings (patch 0025, Vamoto #193) ----

TEST(MotorcycleCost, CityKeysAbsentLeaveTheCostingAsBefore) {
  // No city key: the in-city set is inactive, both flags off, and the
  // profile factors are exactly what they were.
  for (const auto* costing : {"motorcycle", "motorcycle_curvy"}) {
    const Costing c = parse_preferred_costing(costing, R"({"use_highways":0.3,"use_tolls":0.1})");
    EXPECT_FALSE(c.options().has_city_use_highways_case());
    EXPECT_FALSE(c.options().has_city_aversion_case());
    MotorcycleCost cost(c);
    EXPECT_FALSE(cost.city_set_active_);
    EXPECT_FALSE(cost.city_aversion_);
    EXPECT_FALSE(cost.city_fastest_);
    // An inactive set mirrors the profile, so nothing can differ by accident.
    EXPECT_FLOAT_EQ(cost.city_highway_factor_, cost.highway_factor_);
    EXPECT_FLOAT_EQ(cost.city_toll_factor_, cost.toll_factor_);
    EXPECT_FLOAT_EQ(cost.city_surface_factor_, cost.surface_factor_);
  }
}

TEST(MotorcycleCost, AnyCityUseKeyActivatesTheSet) {
  for (const auto* body : {R"({"city_use_highways":1.0})", R"({"city_use_tolls":1.0})",
                           R"({"city_use_ferry":1.0})", R"({"city_use_trails":0.0})"}) {
    MotorcycleCost cost(parse_preferred_costing("motorcycle_curvy", body));
    EXPECT_TRUE(cost.city_set_active_) << body;
  }
}

TEST(MotorcycleCost, CityFactorsUseTheStockFormulas) {
  // city_use_highways 1.0 in a curvy request that avoids motorways outside:
  // the in-city factor equals the one a request with use_highways 1.0 gets.
  MotorcycleCost city(parse_preferred_costing(
      "motorcycle_curvy",
      R"({"use_highways":0.0,"use_tolls":0.0,"city_use_highways":1.0,"city_use_tolls":1.0})"));
  MotorcycleCost open(
      parse_preferred_costing("motorcycle_curvy", R"({"use_highways":1.0,"use_tolls":1.0})"));
  EXPECT_FLOAT_EQ(city.city_highway_factor_, open.highway_factor_);
  EXPECT_FLOAT_EQ(city.city_toll_factor_, open.toll_factor_);
  EXPECT_GT(city.highway_factor_, city.city_highway_factor_);
  EXPECT_GT(city.toll_factor_, city.city_toll_factor_);
}

TEST(MotorcycleCost, CityValuesAreClampedNotSnapped) {
  // Presence is the signal, so an out-of-range value clamps to the edge of
  // the range instead of snapping to a default.
  const Costing c = parse_preferred_costing(
      "motorcycle_curvy",
      R"({"city_use_highways":1.5,"city_use_tolls":-1.0,"city_use_scenic_tolls":0.0})");
  EXPECT_FLOAT_EQ(c.options().city_use_highways(), 1.0f);
  EXPECT_FLOAT_EQ(c.options().city_use_tolls(), 0.0f);
  EXPECT_FLOAT_EQ(c.options().city_use_scenic_tolls(), 0.2f);
}

TEST(MotorcycleCost, CityFlagsParse) {
  MotorcycleCost on(parse_preferred_costing("motorcycle_curvy",
                                            R"({"city_aversion":true,"city_fastest":true})"));
  EXPECT_TRUE(on.city_aversion_);
  EXPECT_TRUE(on.city_fastest_);
  MotorcycleCost off(parse_preferred_costing("motorcycle",
                                             R"({"city_aversion":false,"city_fastest":false})"));
  EXPECT_FALSE(off.city_aversion_);
  EXPECT_FALSE(off.city_fastest_);
}

TEST(MotorcycleCost, CityFerryFollowsTheInCityUseFerry) {
  MotorcycleCost cost(
      parse_preferred_costing("motorcycle", R"({"use_ferry":0.0,"city_use_ferry":0.5})"));
  TestMotorcycleCost neutral(parse_preferred_costing("motorcycle", R"({"use_ferry":0.5})"));
  TestMotorcycleCost avoided(parse_preferred_costing("motorcycle", R"({"use_ferry":0.0})"));
  EXPECT_FLOAT_EQ(cost.city_ferry_factor_, 1.0f);
  EXPECT_FLOAT_EQ(cost.city_ferry_transition_cost_.cost, neutral.ferry_transition_cost_.cost);
  EXPECT_GT(avoided.ferry_transition_cost_.cost, cost.city_ferry_transition_cost_.cost);
}

// ---- forward == reverse transitions (patch 0026, replaces the 0024
// argument-swap test) ----
//
// Both searches hand the pair over in TRAVEL order: the forward search as
// (pred label = earlier edge, edge = later edge), the reverse search as
// (pred = earlier edge, edge = later edge) through the opposing edges
// (bidirectional_astar.cc). The new penalties (0024 ramp hop, 0025/0026
// in-city ferry entry, the city_fastest exemption it no longer has) must
// cost the same in both, or the two trees meet on inconsistent costs.

class TestCurvyCity : public MotorcycleCurvyCost {
public:
  TestCurvyCity(const Costing& c) : MotorcycleCurvyCost(c) {}
  using MotorcycleCurvyCost::city_fastest_scale_;
};

DirectedEdge transition_edge(baldr::RoadClass cls, baldr::Use use, uint32_t density) {
  DirectedEdge e;
  e.set_classification(cls);
  e.set_use(use);
  e.set_density(density);
  return e;
}

struct TransitionPair {
  Cost forward;
  Cost reverse;
};

TransitionPair transition_both_ways(const DynamicCost& cost,
                                    const DirectedEdge& earlier,
                                    const DirectedEdge& later) {
  const NodeInfo node;
  const EdgeLabel pred(0, GraphId(), &earlier, Cost(), 0.0f, sif::TravelMode::kDrive, 0, 0, false,
                       false, InternalTurn::kNoTurn);
  return {cost.TransitionCost(&later, &node, pred, nullptr, nullptr),
          cost.TransitionCostReverse(0, &node, &earlier, &later, nullptr, GraphId(), nullptr, false,
                                     InternalTurn::kNoTurn)};
}

TEST(MotorcycleCurvyCost, NewTransitionPenaltiesAgreeForwardAndReverse) {
  const DirectedEdge motorway_rural = transition_edge(baldr::RoadClass::kMotorway, baldr::Use::kRoad, 4);
  const DirectedEdge ramp_rural = transition_edge(baldr::RoadClass::kPrimary, baldr::Use::kRamp, 4);
  const DirectedEdge motorway_city = transition_edge(baldr::RoadClass::kMotorway, baldr::Use::kRoad, 10);
  const DirectedEdge ramp_city = transition_edge(baldr::RoadClass::kPrimary, baldr::Use::kRamp, 10);
  const DirectedEdge road_city = transition_edge(baldr::RoadClass::kSecondary, baldr::Use::kRoad, 10);
  const DirectedEdge ferry_city = transition_edge(baldr::RoadClass::kPrimary, baldr::Use::kFerry, 10);
  const std::pair<const DirectedEdge*, const DirectedEdge*> hops[] = {
      {&motorway_rural, &ramp_rural}, {&ramp_rural, &motorway_rural},
      {&motorway_city, &ramp_city},   {&ramp_city, &motorway_city},
      {&road_city, &ferry_city},      {&ferry_city, &road_city},
  };
  for (const auto* body :
       {R"({})", R"({"use_ferry":0.0,"city_use_ferry":0.5})",
        R"({"city_fastest":true,"city_use_highways":1.0,"city_use_ferry":1.0})",
        R"({"shortest":true,"use_ferry":0.0,"city_use_ferry":1.0})"}) {
    for (const auto* costing : {"motorcycle", "motorcycle_curvy"}) {
      const Costing c = parse_preferred_costing(costing, body);
      const cost_ptr_t cost = std::string(costing) == "motorcycle" ? CreateMotorcycleCost(c)
                                                                   : CreateMotorcycleCurvyCost(c);
      for (const auto& [earlier, later] : hops) {
        const TransitionPair t = transition_both_ways(*cost, *earlier, *later);
        EXPECT_FLOAT_EQ(t.forward.cost, t.reverse.cost) << costing << " " << body;
        EXPECT_FLOAT_EQ(t.forward.secs, t.reverse.secs) << costing << " " << body;
      }
    }
  }
}

TEST(MotorcycleCurvyCost, RampPenaltyAppliesInACityWithCityFastest) {
  // Patch 0026 (F1): the in-city ramp hop pays the curvy ramp penalty with
  // setting (2) on as well -- the curvy transition exceeds the stock one by
  // exactly kCurvyHighwayRampPenalty.
  const DirectedEdge motorway_city = transition_edge(baldr::RoadClass::kMotorway, baldr::Use::kRoad, 10);
  const DirectedEdge ramp_city = transition_edge(baldr::RoadClass::kPrimary, baldr::Use::kRamp, 10);
  const char* body = R"({"city_fastest":true,"city_use_highways":1.0})";
  const cost_ptr_t curvy = CreateMotorcycleCurvyCost(parse_preferred_costing("motorcycle_curvy", body));
  const cost_ptr_t stock = CreateMotorcycleCost(parse_preferred_costing("motorcycle", body));
  for (const auto& [earlier, later] : {std::pair{&motorway_city, &ramp_city},
                                       std::pair{&ramp_city, &motorway_city}}) {
    const TransitionPair c = transition_both_ways(*curvy, *earlier, *later);
    const TransitionPair m = transition_both_ways(*stock, *earlier, *later);
    EXPECT_FLOAT_EQ(c.forward.cost - m.forward.cost, kCurvyHighwayRampPenalty);
    EXPECT_FLOAT_EQ(c.reverse.cost - m.reverse.cost, kCurvyHighwayRampPenalty);
  }
}

TEST(MotorcycleCost, CityFerryEntryLeavesNoResidueUnderShortest) {
  // Patch 0026 (L1): a shortest search ignores penalties, so the in-city
  // ferry swap must not add or subtract cost there.
  const DirectedEdge road_city = transition_edge(baldr::RoadClass::kSecondary, baldr::Use::kRoad, 10);
  const DirectedEdge ferry_city = transition_edge(baldr::RoadClass::kPrimary, baldr::Use::kFerry, 10);
  const cost_ptr_t swapped = CreateMotorcycleCost(parse_preferred_costing(
      "motorcycle", R"({"shortest":true,"use_ferry":0.0,"city_use_ferry":1.0})"));
  const cost_ptr_t plain =
      CreateMotorcycleCost(parse_preferred_costing("motorcycle", R"({"shortest":true})"));
  const TransitionPair a = transition_both_ways(*swapped, road_city, ferry_city);
  const TransitionPair b = transition_both_ways(*plain, road_city, ferry_city);
  EXPECT_FLOAT_EQ(a.forward.cost, b.forward.cost);
  EXPECT_FLOAT_EQ(a.reverse.cost, b.reverse.cost);
}

TEST(MotorcycleCurvyCost, CityFastestScaleIsTheStraightMainRoadCharge) {
  // Patch 0026 (F1): sport touring's alpha 0.6 -> (1 + 0.6 * 1.5) * 1.76.
  TestCurvyCity sport(parse_preferred_costing("motorcycle_curvy",
                                              R"({"curvy_alpha":0.6,"city_fastest":true})"));
  EXPECT_NEAR(sport.city_fastest_scale_, 1.9f * 1.76f, 1e-4);
  TestCurvyCity twisty(parse_preferred_costing("motorcycle_curvy",
                                               R"({"curvy_alpha":0.95,"city_fastest":true})"));
  EXPECT_GT(twisty.city_fastest_scale_, sport.city_fastest_scale_);
}

} // namespace

#endif
