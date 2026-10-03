#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/pointll.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

void check_locations(const size_t location_count, const size_t max_locations) {
  // check that location size does not exceed max.
  if (location_count > max_locations) {
    throw valhalla_exception_t{150, std::to_string(max_locations)};
  };
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float max_distance,
                    bool all_pairs) {
  // test if total distance along a polyline formed by connecting locations exceeds the maximum
  // or if all_pairs is specified test all pairs of locations to see if any are over the threshold
  float total_path_distance = 0.0f;
  for (int i = 0; i < locations.size(); ++i) {
    for (int j = i + 1; j < locations.size(); ++j) {
      auto dist = to_ll(locations.Get(i)).Distance(to_ll(locations.Get(j)));
      total_path_distance += i + 1 == j ? dist : 0;
      if ((!all_pairs && total_path_distance > max_distance) || (all_pairs && dist > max_distance))
        throw valhalla_exception_t{154,
                                   std::to_string(static_cast<size_t>(max_distance)) + " meters"};
      if (!all_pairs)
        break;
    }
  }
}

// Gate locations (patch 0030, Vamoto #209). A gate is a line across the
// route: it correlates to every directed edge whose shape crosses the line in
// the gate's direction of travel, at the crossing point. The leg then ends on
// whichever road crosses it cheapest -- "somewhere across here", not a point
// -- and, as a through location, the next leg continues on that same edge in
// the same direction, so a joint is never a U-turn.
//
// Excluded: shortcuts, edges the costing does not allow, edges outside the
// location's search_filter road classes, and edges entering a dead-end region
// (`not_thru`, set by the tile builder): riding into a stub to cross a gate
// and back out is exactly the joint U-turn this replaces. Those dead-end
// crossings are kept as filtered edges, the engine's second-pass fallback.
constexpr double kMaxGateRadius = 30000.0;
constexpr double kGatePi = 3.14159265358979323846;

bool class_filtered(const DirectedEdge* edge, const valhalla::SearchFilter& filter) {
  const auto rc = static_cast<uint32_t>(edge->classification());
  return rc > static_cast<uint32_t>(filter.min_road_class()) ||
         rc < static_cast<uint32_t>(filter.max_road_class());
}

void correlate_gate(valhalla::Location& loc, GraphReader& reader, const sif::cost_ptr_t& costing) {
  const double lat0 = loc.ll().lat(), lng0 = loc.ll().lng();
  const double mx = 111320.0 * std::cos(lat0 * kGatePi / 180.0), my = 110574.0;
  const double h = loc.gate_heading() * kGatePi / 180.0;
  const double tx = std::sin(h), ty = std::cos(h); // travel direction (east, north)
  const double rx = ty, ry = -tx;                   // to the right of travel
  const double half = std::min<double>(loc.gate_radius(), kMaxGateRadius);
  // The gate in local metres around ll: g1 + u * (g2 - g1), u in [0, 1].
  const double g1x = -half * rx, g1y = -half * ry, gdx = 2 * half * rx, gdy = 2 * half * ry;
  const PointLL p1(lng0 + g1x / mx, lat0 + g1y / my);
  const PointLL p2(lng0 + (g1x + gdx) / mx, lat0 + (g1y + gdy) / my);

  const auto& level = TileHierarchy::levels().back();
  const auto bins = level.tiles.Intersect(std::vector<PointLL>{p1, p2});
  const auto& filter = loc.search_filter();
  std::unordered_set<uint64_t> seen;
  google::protobuf::RepeatedPtrField<valhalla::PathEdge> edges, dead_ends;

  // One crossing of a shape (stored orientation) at fraction `along` of its
  // length; `with_shape` = the travel along the stored shape crosses the gate
  // in the gate's direction.
  auto add = [&](GraphId shape_edge_id, const DirectedEdge* shape_edge, const graph_tile_ptr& tile,
                 bool with_shape, double along, const PointLL& at, double heading_deg) {
    // The directed edge that travels along the stored shape is the one with
    // forward() == true.
    GraphId id = shape_edge_id;
    const DirectedEdge* edge = shape_edge;
    graph_tile_ptr edge_tile = tile;
    if (shape_edge->forward() != with_shape) {
      id = reader.GetOpposingEdgeId(shape_edge_id, edge, edge_tile);
      if (!id.is_valid() || !edge) {
        return;
      }
    }
    if (!seen.insert(id.value).second) {
      return;
    }
    if (edge->is_shortcut() || !costing->Allowed(edge, edge_tile, sif::kDisallowShortcut) ||
        class_filtered(edge, filter)) {
      return;
    }
    valhalla::PathEdge pe;
    pe.set_graph_id(id);
    const double pct = with_shape ? along : 1.0 - along;
    pe.set_percent_along(std::clamp(pct, 0.0, 1.0));
    pe.mutable_ll()->set_lat(at.lat());
    pe.mutable_ll()->set_lng(at.lng());
    pe.set_distance(0);
    pe.set_heading(static_cast<float>(heading_deg));
    pe.set_side_of_street(valhalla::Location::kNone);
    // Every crossing counts as reachable: the reach retry must not drop them.
    pe.set_inbound_reach(1 << 20);
    pe.set_outbound_reach(1 << 20);
    (edge->not_thru() ? dead_ends : edges).Add(std::move(pe));
  };

  for (const auto& [tileid, bin_ids] : bins) {
    auto bin_tile = reader.GetGraphTile(GraphId(tileid, level.level, 0));
    if (!bin_tile) {
      continue;
    }
    for (const auto bin : bin_ids) {
      for (const auto& bin_edge : bin_tile->GetBin(bin)) {
        // Bins list edges of other tiles too; a region pack may not hold
        // them (see patch 0027): skip what is not loaded.
        graph_tile_ptr tile = bin_tile;
        if (!reader.GetGraphTile(bin_edge, tile)) {
          continue;
        }
        const DirectedEdge* de = tile->directededge(bin_edge);
        if (de->is_shortcut()) {
          continue;
        }
        const auto info = tile->edgeinfo(de);
        const auto& shape = info.shape();
        double total = 0;
        for (size_t i = 0; i + 1 < shape.size(); ++i) {
          total += shape[i].Distance(shape[i + 1]);
        }
        if (total <= 0) {
          continue;
        }
        double acc = 0;
        for (size_t i = 0; i + 1 < shape.size(); ++i) {
          const double ax = (shape[i].lng() - lng0) * mx, ay = (shape[i].lat() - lat0) * my;
          const double bx = (shape[i + 1].lng() - lng0) * mx, by = (shape[i + 1].lat() - lat0) * my;
          const double sdx = bx - ax, sdy = by - ay;
          const double seg = shape[i].Distance(shape[i + 1]);
          const double den = sdx * gdy - sdy * gdx;
          if (std::abs(den) > 1e-9) {
            // a + t * sd == g1 + u * gd
            const double t = ((g1x - ax) * gdy - (g1y - ay) * gdx) / den;
            const double u = ((g1x - ax) * sdy - (g1y - ay) * sdx) / den;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
              const double dot = sdx * tx + sdy * ty;
              if (dot != 0) {
                const PointLL at(shape[i].lng() + t * (shape[i + 1].lng() - shape[i].lng()),
                                 shape[i].lat() + t * (shape[i + 1].lat() - shape[i].lat()));
                const bool with_shape = dot > 0;
                double hd = std::atan2(sdx, sdy) * 180.0 / kGatePi;
                if (!with_shape) {
                  hd += 180.0;
                }
                hd = std::fmod(hd + 360.0, 360.0);
                add(bin_edge, de, tile, with_shape, (acc + t * seg) / total, at, hd);
                break; // one crossing per edge is enough
              }
            }
          }
          acc += seg;
        }
      }
    }
  }

  if (edges.empty() && dead_ends.empty()) {
    return; // nothing crosses: keep the plain point correlation
  }
  auto* corr = loc.mutable_correlation();
  corr->mutable_edges()->Clear();
  corr->mutable_filtered_edges()->Clear();
  if (edges.empty()) {
    // Only dead-end crossings: route to them rather than fail.
    corr->mutable_edges()->Swap(&dead_ends);
  } else {
    corr->mutable_edges()->Swap(&edges);
    corr->mutable_filtered_edges()->Swap(&dead_ends);
  }
}

} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_route(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations(), request);
  // need to check location size here instead of in parse_locations because of locate action needing
  // a different size
  if (request.options().locations_size() < 2) {
    throw valhalla_exception_t{120};
  };
  parse_costing(request);
}

void loki_worker_t::route(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  init_route(request);
  auto& options = *request.mutable_options();
  const auto& costing_name = Costing_Enum_Name(options.costing_type());
  if (request.options().action() == Options::centroid) {
    check_locations(options.locations_size(), max_locations.find("centroid")->second);
    check_distance(options.locations(), max_distance.find("centroid")->second, true);
  } else {
    check_locations(options.locations_size(), max_locations.find(costing_name)->second);
    check_distance(options.locations(), max_distance.find(costing_name)->second, false);
  }

  // check distance for hierarchy pruning
  check_hierarchy_distance(request);

  auto connectivity_level = TileHierarchy::levels().back();
  uint32_t connectivity_radius = 0;
  // Validate walking distances (make sure they are in the accepted range)
  if (costing_name == "multimodal" || costing_name == "transit" ||
      costing_name == "auto_pedestrian") {
    auto& ped_opts = *options.mutable_costings()->find(Costing::pedestrian)->second.mutable_options();

    // "transit_start_end_max_distance" is deprecated
    // but we will still allow it for some time
    if (ped_opts.has_transit_transfer_max_distance_case()) {
      ped_opts.set_multimodal_start_end_max_distance(ped_opts.transit_start_end_max_distance());
    }

    // we have renamed this parameter
    if (!ped_opts.has_multimodal_start_end_max_distance_case())
      ped_opts.set_multimodal_start_end_max_distance(min_multimodal_walking_dist);
    auto multimodal_start_end_max_distance = ped_opts.multimodal_start_end_max_distance();

    if (!ped_opts.has_transit_transfer_max_distance_case())
      ped_opts.set_transit_transfer_max_distance(min_multimodal_walking_dist);
    auto transit_transfer_max_distance = ped_opts.transit_transfer_max_distance();

    if (multimodal_start_end_max_distance < min_multimodal_walking_dist ||
        multimodal_start_end_max_distance > max_multimodal_walking_dist) {
      throw valhalla_exception_t{155, " Min: " + std::to_string(min_multimodal_walking_dist) +
                                          " Max: " + std::to_string(max_multimodal_walking_dist) +
                                          " (Meters)"};
    }
    if (transit_transfer_max_distance < min_multimodal_walking_dist ||
        transit_transfer_max_distance > max_multimodal_walking_dist) {
      throw valhalla_exception_t{156, " Min: " + std::to_string(min_multimodal_walking_dist) +
                                          " Max: " + std::to_string(max_multimodal_walking_dist) +
                                          " (Meters)"};
    }
    if (costing_name != "auto_pedestrian") {
      connectivity_level = TileHierarchy::GetTransitLevel();
    }
    connectivity_radius = ped_opts.transit_start_end_max_distance();
  }

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    auto* locations = options.mutable_locations();
    locations->begin()->set_minimum_inbound_reachability(0);
    locations->rbegin()->set_minimum_outbound_reachability(0);
    auto locations_size = locations->size();

    // maybe squeeze in the first and last locations of each user specified feature for cost factor
    // lines as we'll need those for edge walking
    for (const auto& line : options.cost_factor_lines()) {
      google::protobuf::RepeatedPtrField<Location> first_and_last;
      first_and_last.Add()->CopyFrom(*line.shape().begin());
      first_and_last.Add()->CopyFrom(*line.shape().rbegin());
      Api dummy;
      parse_locations(&first_and_last, dummy);
      locations->MergeFrom(first_and_last);
    }

    // in case of auto_pedestrian costing, we 1) only allow two locations
    // and 2) need two different costings for the start and end location.
    // Search::search does not allow for multiple costings per location so instead
    // we bite the bullet and call search twice, merging the results.
    // TODO(chris): right now we hash location based purely on ll, but what if the user wants to start
    // and end at the same location but with different costing? What does that even mean? Find the
    // nearest parking and walk back to where I am? Seems like a plausible use case...
    if (costing_name == "auto_pedestrian") {
      if (locations_size > 2) {
        throw valhalla_exception_t{150, "for auto_pedestrian: " + std::to_string(locations->size())};
      }
      google::protobuf::RepeatedPtrField<Location> start_loc(locations->begin(),
                                                             locations->begin() + 1);
      search_.search(start_loc, mode_costing[static_cast<size_t>(mode)]);
      google::protobuf::RepeatedPtrField<Location> end_loc(locations->begin() + 1,
                                                           locations->begin() + 2);
      search_.search(end_loc, mode_costing[static_cast<size_t>(mode)]);
      // merge them again
      locations->at(0).CopyFrom(start_loc.at(0));
      locations->at(1).CopyFrom(end_loc.at(0));
    } else {
      search_.search(*locations, mode_costing[static_cast<size_t>(mode)]);
    }

    // Gate locations (patch 0030) replace their point correlation with every
    // edge crossing the gate line.
    for (int i = 0; i < locations_size; ++i) {
      auto& location = locations->at(i);
      if (location.has_gate_heading_case() && location.has_gate_radius_case()) {
        correlate_gate(location, *reader, mode_costing[static_cast<size_t>(mode)]);
      }
    }

    // throw if there's a location we did not find any
    // candidates for
    for (const auto& location : *locations) {
      if (location.correlation().edges().empty() && location.correlation().filtered_edges().empty()) {
        throw valhalla_exception_t(171);
      }
    }

    if (connectivity_map) {
      for (int i = 0; i < locations_size; ++i) {
        auto colors =
            connectivity_map->get_colors(connectivity_level, locations->at(i), connectivity_radius);
        for (auto color : colors) {
          auto itr = color_counts.find(color);
          if (itr == color_counts.cend()) {
            color_counts[color] = 1;
          } else {
            ++itr->second;
          }
        }
      }
    }

    // store the correlations for the cost factor lines
    // todo(chris): make sure this'll work with auto_pedestrian as well
    size_t i = 0;
    for (auto& line : *options.mutable_cost_factor_lines()) {
      size_t correlated_start_index = locations_size + 2 * i;
      line.mutable_locations()->Add(std::move(locations->at(correlated_start_index)));
      size_t correlated_end_index = locations_size + 2 * i + 1;
      line.mutable_locations()->Add(std::move(locations->at(correlated_end_index)));
      ++i;
    }
    // and remove the first and last cost factor lines from the locations again
    locations->DeleteSubrange(locations_size, locations->size() - locations_size);

  } catch (const valhalla_exception_t& e) { throw e; } catch (const std::exception&) {
    throw valhalla_exception_t{171};
  }

  // are all the locations in the same color regions
  if (!connectivity_map) {
    return;
  }
  bool connected = false;
  for (const auto& c : color_counts) {
    if (c.second == static_cast<size_t>(options.locations_size())) {
      connected = true;
      break;
    }
  }
  if (!connected) {
    throw valhalla_exception_t{170};
  };
}
} // namespace loki
} // namespace valhalla
