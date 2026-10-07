#pragma once

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/expansion.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr size_t kInterruptIterationsInterval = 5000;

enum class ExpansionType { forward = 0, reverse = 1, multimodal = 2 };

/**
 * Pure virtual class defining the interface for PathAlgorithm - the algorithm
 * to create shortest path.
 */
class PathAlgorithm {
public:
  /**
   * Constructor
   */
  PathAlgorithm(uint32_t max_reserved_labels_count, bool clear_reserved_memory)
      : interrupt(nullptr), has_ferry_(false), not_thru_pruning_(true), expansion_callback_(),
        max_reserved_labels_count_(max_reserved_labels_count),
        clear_reserved_memory_(clear_reserved_memory) {
  }

  PathAlgorithm(const PathAlgorithm&) = delete;
  PathAlgorithm& operator=(const PathAlgorithm&) = delete;

  /**
   * Destructor
   */
  virtual ~PathAlgorithm() {
  }

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) = 0;

  /**
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  virtual const char* name() const = 0;

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() = 0;

  /**
   * Set a callback that will throw when the path computation should be aborted
   * @param interrupt_callback  the function to periodically call to see if
   *                            we should abort
   */
  void set_interrupt(const std::function<void()>* interrupt_callback) {
    interrupt = interrupt_callback;
  }

  /**
   * Does the path include a ferry?
   * @return  Returns true if the path includes a ferry.
   */
  bool has_ferry() const {
    return has_ferry_;
  }

  /**
   * Patch 0049: did the last GetBestPath stop because it crossed the
   * request's search label cap (sif::DynamicCost::SearchLabelCap)? Such a
   * search returns no path, and route_action turns it into error 447.
   */
  bool search_budget_exhausted() const {
    return search_budget_exhausted_;
  }

  /**
   *
   * There is a rare case where we may encounter only_restrictions with edges being
   * marked as not_thru.  Basically the only way to get in this area is via one edge
   * and all other edges are restricted, but this one edge is also marked as not_thru.
   * Therefore, on the first pass the expansion stops as we cannot take the restricted
   * turns and we cannot go into the not_thru region. On the 2nd pass, we now ignore
   * not_thru flags and allow entry into the not_thru region due to the fact that
   * not_thru_pruning_ is false.  See the gurka test not_thru_pruning_.
   *
   * Set the not_thru_pruning_
   * @param pruning  set the not_thru_pruning_ to pruning value.
   *                 only set on the second pass
   */
  void set_not_thru_pruning(const bool pruning) {
    not_thru_pruning_ = pruning;
  }

  /**
   * Get the not thru pruning
   * @return  Returns not_thru_pruning_
   */
  bool not_thru_pruning() {
    return not_thru_pruning_;
  }

  /**
   * Sets the functor which will track the algorithms expansion.
   *
   * @param  expansion_callback  the functor to call back when the algorithm makes progress
   *                             on a given edge
   */
  using expansion_callback_t = std::function<void(baldr::GraphReader&,
                                                  const baldr::GraphId,
                                                  const baldr::GraphId,
                                                  const char*,
                                                  const Expansion_EdgeStatus,
                                                  float,
                                                  uint32_t,
                                                  float,
                                                  const Expansion_ExpansionType,
                                                  const uint8_t,
                                                  const TravelMode)>;
  void set_track_expansion(const expansion_callback_t& expansion_callback) {
    expansion_callback_ = expansion_callback;
  }

protected:
  const std::function<void()>* interrupt;

  bool has_ferry_; // Indicates whether the path has a ferry

  bool not_thru_pruning_; // Indicates whether to allow access into a not-thru region.

  // for tracking the expansion of the algorithm visually
  expansion_callback_t expansion_callback_;

  // when doing timezone differencing a timezone cache speeds up the computation
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  uint32_t max_reserved_labels_count_;

  // if `true` clean reserved memory for edge labels
  bool clear_reserved_memory_;

  // Patch 0049: one expansion adds at most a node's edges on every level
  // (< 400); 4096 covers both directions of one loop turn.
  static constexpr uint32_t kLabelCapSlack = 4096;
  // Patch 0049: the labels to reserve for one direction of a search: the
  // configured count, never more than the request's cap plus the slack the
  // `>` cap check can overshoot by (cap 0 = no cap).
  static uint32_t LabelReservation(uint32_t configured, uint32_t cap) {
    return cap == 0 ? configured
                    : static_cast<uint32_t>(
                          std::min<uint64_t>(configured, uint64_t{cap} + kLabelCapSlack));
  }

  // Patch 0049: set when the last search crossed its label cap.
  bool search_budget_exhausted_ = false;
};

/**
 * Check for path completion along the same edge. Edge ID in question
 * is along both an origin and destination and origin shows up at the
 * beginning of the edge while the destination shows up at the end of
 * the edge.
 * @param  edgeid       Edge id.
 * @param  origin       Origin path location information.
 * @param  destination  Destination path location information.
 */
inline bool IsTrivial(const baldr::GraphId& edgeid,
                      const valhalla::Location& origin,
                      const valhalla::Location& destination) {
  for (const auto& destination_edge : destination.correlation().edges()) {
    if (destination_edge.graph_id() == edgeid) {
      for (const auto& origin_edge : origin.correlation().edges()) {
        if (origin_edge.graph_id() == edgeid &&
            origin_edge.percent_along() <= destination_edge.percent_along()) {
          return true;
        }
      }
    }
  }
  return false;
}

// Container for the data we iterate over in Expand* function
struct EdgeMetadata {
  const baldr::DirectedEdge* edge;
  baldr::GraphId edge_id;
  EdgeStatusInfo* edge_status;

  inline static EdgeMetadata make(const baldr::GraphId& node,
                                  const baldr::NodeInfo* nodeinfo,
                                  const baldr::graph_tile_ptr& tile,
                                  EdgeStatus& edge_status_) {
    baldr::GraphId edge_id = {node.tileid(), node.level(), nodeinfo->edge_index()};
    EdgeStatusInfo* edge_status = edge_status_.GetPtr(edge_id, tile);
    const baldr::DirectedEdge* directededge = tile->directededge(edge_id);
    return {directededge, edge_id, edge_status};
  }

  inline EdgeMetadata& operator++() {
    ++edge;
    ++edge_id;
    ++edge_status;
    return *this;
  }

  inline operator bool() const {
    return edge;
  }

  inline bool operator!() const {
    return !edge;
  }
};

} // namespace thor
} // namespace valhalla
