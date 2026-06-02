// valhalla_extract_stretches
//
// Layer 1 of the better_mc_routing v2 scenic-routing pipeline (Issue 02,
// Slice 1 of the v2 PRD). Reads a built Valhalla tileset and emits a single
// flat protobuf sidecar `stretches.bin` containing the scenic-stretch
// catalog.
//
// Pipeline:
//   1. Walk every (non-shortcut, forward-direction) DirectedEdge in level-2 tiles
//      (level 0 + 1 are shortcuts / arterials — we want the base curvy roads).
//   2. Skip edges that fail seed eligibility (sinuosity_byte < kSeed, restricted,
//      etc.).
//   3. Grow forward through neighbors via NodeInfo::edge_index — stop on
//      class change, restricted access, roundabout, or sinuosity below grow
//      threshold (with straight-blip budget).
//   4. Hand the candidate run to mjolnir::emit_with_splits which handles the
//      [1 km, 20 km] band and lowest-sinuosity splits.
//   5. Write all emitted stretches as protobuf to <output_dir>/stretches.bin.
//
// Backwards growth is implicit: the walk seeds at every edge that passes the
// seed test, so two stretches that should join up will both be found and
// (because the chain is deterministic in tile order) emitted as one. We DO
// guard against re-emitting the same chain by marking visited edges in a
// per-thread set.

#include "argparse_utils.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/nodeinfo.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "mjolnir/stretch_extractor.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "valhalla/proto/stretches.pb.h"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

// Decode an edge's shape from EdgeInfo's encoded polyline. Returns empty on
// failure. Reversed if the edge is not in the forward direction.
std::vector<PointLL> get_edge_shape(const GraphTile& tile, const DirectedEdge* de) {
  auto ei = tile.edgeinfo(de);
  auto shape = ei.shape();
  if (!de->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  return shape;
}

// Build an EdgeCandidate from a tile + DirectedEdge.
EdgeCandidate make_candidate(const GraphTile& tile, const DirectedEdge* de) {
  EdgeCandidate c;
  c.length_m = de->length();
  c.sinuosity_byte = tile.edgeinfo(de).sinuosity();
  c.road_class = de->classification();
  c.use = de->use();
  c.roundabout = de->roundabout();
  // "Restricted access" for our purposes = the edge has any access_restriction
  // attached. We don't sub-filter — anything restricted is a hard stop.
  c.restricted_access = de->access_restriction() != 0;
  c.shape = get_edge_shape(tile, de);
  return c;
}

// Find the NEXT directed edge in the forward growth chain.
//
// At the end node we want the unique outbound edge that:
//   - is not the opposite (would walk back)
//   - matches the anchor class
//   - is not a roundabout, not restricted, not a shortcut
//
// If there are 0 or 2+ candidates we stop (an intersection breaks growth).
// Returns an invalid GraphId on stop.
GraphId next_edge_in_chain(GraphReader& reader,
                           const graph_tile_ptr& current_tile,
                           const DirectedEdge* current_de,
                           const GraphId& /*current_edge_id*/,
                           RoadClass anchor_class) {
  const GraphId end_node = current_de->endnode();
  // If the end node lives in the same tile we're already holding, reuse the
  // existing ptr instead of round-tripping through the GraphReader cache.
  graph_tile_ptr end_tile_ptr = (end_node.tile_base() == current_tile->id())
                                     ? current_tile
                                     : reader.GetGraphTile(end_node);
  if (!end_tile_ptr) {
    return {};
  }
  const NodeInfo* node = end_tile_ptr->node(end_node);
  // The opposite of `current_de` lives at end_node and we want to skip it.
  const uint32_t opp_local_idx = current_de->opp_local_idx();

  GraphId candidate{};
  uint32_t found = 0;
  const uint32_t base_idx = node->edge_index();
  for (uint32_t i = 0; i < node->edge_count(); ++i) {
    const DirectedEdge* de = end_tile_ptr->directededge(base_idx + i);
    if (de->shortcut()) continue;
    if (de->localedgeidx() == opp_local_idx) continue; // would walk back
    if (de->roundabout()) continue;
    if (de->classification() != anchor_class) continue;
    if (de->access_restriction() != 0) continue;
    // Found one. Stop counting at 2 to keep this O(1) per node.
    if (++found > 1) {
      return {};
    }
    candidate = GraphId(end_node.tileid(), end_node.level(), base_idx + i);
  }
  if (found != 1) return {};
  return candidate;
}

// Walk forward from a seed and collect candidate edges until growth must stop.
// Tracks the straight-blip budget per mjolnir::stretch_extractor.h constants.
std::vector<EdgeCandidate> grow_forward(GraphReader& reader,
                                        graph_tile_ptr seed_tile,
                                        const GraphId& seed_edge_id,
                                        std::unordered_set<uint64_t>& visited) {
  std::vector<EdgeCandidate> chain;
  const DirectedEdge* seed_de = seed_tile->directededge(seed_edge_id);
  const RoadClass anchor_class = seed_de->classification();

  GraphId cur_edge_id = seed_edge_id;
  graph_tile_ptr cur_tile = seed_tile;
  const DirectedEdge* cur_de = seed_de;

  uint32_t straight_count = 0;
  uint32_t straight_meters = 0;

  while (true) {
    if (visited.count(cur_edge_id.value)) break;
    visited.insert(cur_edge_id.value);
    chain.push_back(make_candidate(*cur_tile, cur_de));

    // For non-seed edges, decide whether the straight-blip budget is busted.
    // (The seed itself passed the seed test, so it's never a blip.)
    if (chain.size() > 1) {
      const auto& last = chain.back();
      if (last.sinuosity_byte < kGrowSinuosityByte) {
        straight_count++;
        straight_meters += last.length_m;
        if (straight_count > kMaxStraightBlipCount ||
            straight_meters > kMaxStraightBlipMeters) {
          // Drop this last edge — it busted the budget.
          chain.pop_back();
          break;
        }
      }
    }

    GraphId next = next_edge_in_chain(reader, cur_tile, cur_de, cur_edge_id, anchor_class);
    if (!next.is_valid()) break;
    graph_tile_ptr next_tile = (next.tile_base() == cur_tile->id())
                                   ? cur_tile
                                   : reader.GetGraphTile(next);
    if (!next_tile) break;
    cur_tile = next_tile;
    cur_de = next_tile->directededge(next);
    cur_edge_id = next;
  }
  return chain;
}

// Write a GeoJSON FeatureCollection of `collection` to `path`. Each Feature is
// a LineString in [lon, lat] order (per RFC 7946) with the slice-7 QA
// properties: stretch_id, length_km, score, road_class, mean_sinuosity.
//
// Single-line JSON, ASCII only, six decimals of coordinate precision (~11 cm
// at the equator). No external JSON dependency — the schema is small enough
// to write by hand, and pulling in nlohmann/rapidjson here would bloat the
// build for a one-off serialization.
//
// Used by extract_stretches as the QA companion to stretches.bin (Slice 7
// of the v2 PRD). Layer 0 of the toolchain stays binary-protobuf; this exists
// only so reviewers can drop the output into uMap / QGIS / geopandas without
// writing a custom proto reader.
void write_geojson(const valhalla::StretchCollection& collection,
                   const std::filesystem::path& path) {
  std::ofstream geojson(path, std::ios::trunc);
  if (!geojson) {
    throw std::runtime_error("Failed to open output file: " + path.string());
  }
  // Six decimals matches what we serialize in protobuf (doubles), and keeps
  // each line short for uMap-style preview tools.
  geojson << std::fixed << std::setprecision(6);
  geojson << R"({"type":"FeatureCollection","features":[)";
  bool first = true;
  for (int i = 0; i < collection.stretches_size(); ++i) {
    const auto& s = collection.stretches(i);
    if (!first) geojson << ",";
    first = false;
    geojson << R"({"type":"Feature","geometry":{"type":"LineString","coordinates":[)";
    const int n = s.polyline_lat_size();
    for (int j = 0; j < n; ++j) {
      if (j > 0) geojson << ",";
      // GeoJSON ordering is [lon, lat], NOT [lat, lon] — common foot-gun, and
      // worth a re-read at any future edit.
      geojson << "[" << s.polyline_lon(j) << "," << s.polyline_lat(j) << "]";
    }
    geojson << R"(]},"properties":{)";
    geojson << R"("stretch_id":)" << s.id()
            << R"(,"length_km":)" << s.length_km()
            << R"(,"score":)" << s.score()
            << R"(,"road_class":)" << s.road_class()
            << R"(,"mean_sinuosity":)" << s.mean_sinuosity();
    geojson << "}}";
  }
  geojson << "]}\n";
  if (!geojson) {
    throw std::runtime_error("Failed to write GeoJSON: " + path.string());
  }
}

// Walk all base-level tiles and emit stretches.
void extract_stretches(const boost::property_tree::ptree& pt,
                       const std::filesystem::path& output_dir) {
  GraphReader reader(pt.get_child("mjolnir"));

  // We extract only on the most-detailed (last) hierarchy level — base roads.
  const auto& levels = TileHierarchy::levels();
  const uint8_t base_level = levels.back().level;

  // Visited set: one entry per directed-edge GraphId we've already pulled into
  // a chain. Stops re-walking.
  std::unordered_set<uint64_t> visited;
  visited.reserve(1 << 20);

  valhalla::StretchCollection collection;
  uint32_t next_id = 0;

  const auto& tiles = levels.back().tiles;
  uint32_t tile_count = 0;
  uint32_t seed_count = 0;
  uint32_t emitted_count = 0;

  for (uint32_t tile_index = 0; tile_index < tiles.TileCount(); ++tile_index) {
    GraphId tile_id(tile_index, base_level, 0);
    if (!reader.DoesTileExist(tile_id)) continue;
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    if (!tile) continue;
    tile_count++;

    const uint32_t n_nodes = tile->header()->nodecount();
    GraphId node_id = tile_id;
    for (uint32_t n = 0; n < n_nodes; ++n, ++node_id) {
      const NodeInfo* node = tile->node(node_id);
      const uint32_t base_idx = node->edge_index();
      for (uint32_t i = 0; i < node->edge_count(); ++i) {
        const uint32_t local_idx = base_idx + i;
        GraphId edge_id(tile_id.tileid(), base_level, local_idx);
        const DirectedEdge* de = tile->directededge(local_idx);

        // Seed eligibility checks.
        if (de->shortcut()) continue;
        if (!de->forward()) continue; // visit each pair once
        if (de->roundabout()) continue;
        if (de->access_restriction() != 0) continue;
        if (de->classification() == RoadClass::kInvalid) continue;

        const uint8_t byte = tile->edgeinfo(de).sinuosity();
        if (byte < kSeedSinuosityByte) continue;

        if (visited.count(edge_id.value)) continue;
        seed_count++;

        auto chain = grow_forward(reader, tile, edge_id, visited);
        if (chain.empty()) continue;

        auto stretches = emit_with_splits(std::span<const EdgeCandidate>(chain));
        for (const auto& s : stretches) {
          auto* msg = collection.add_stretches();
          msg->set_id(next_id++);
          msg->set_length_km(s.length_km);
          msg->set_score(s.score);
          msg->set_road_class(static_cast<uint32_t>(s.road_class));
          msg->set_mean_sinuosity(s.mean_sinuosity_raw);
          for (const auto& p : s.polyline) {
            msg->add_polyline_lat(p.lat());
            msg->add_polyline_lon(p.lng());
          }
          emitted_count++;
        }
      }
    }
  }

  LOG_INFO("extract_stretches: scanned " + std::to_string(tile_count) +
           " tiles, " + std::to_string(seed_count) + " seeds, emitted " +
           std::to_string(emitted_count) + " stretches.");

  std::filesystem::create_directories(output_dir);
  const auto out_path = output_dir / "stretches.bin";
  std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("Failed to open output file: " + out_path.string());
  }
  if (!collection.SerializeToOstream(&out)) {
    throw std::runtime_error("Failed to serialize stretches.bin");
  }
  LOG_INFO("Wrote " + out_path.string() + " (" +
           std::to_string(collection.stretches_size()) + " stretches).");

  // Slice 7 QA dump: write a GeoJSON FeatureCollection alongside the
  // protobuf so reviewers can inspect the catalog visually in uMap or QGIS
  // without writing a custom viewer. See write_geojson for the schema
  // contract.
  const auto geojson_path = output_dir / "stretches.geojson";
  write_geojson(collection, geojson_path);
  LOG_INFO("Wrote " + geojson_path.string() + " (GeoJSON QA dump).");
}

} // namespace

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  std::string output_dir_str;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "Extract scenic stretches from built Valhalla tiles into a protobuf sidecar.\n"
      "Layer 1 of the better_mc_routing v2 scenic-routing pipeline.\n\n");

    options.add_options()
      ("h,help", "Print this help message")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("o,output-dir", "Output directory for stretches.bin",
        cxxopts::value<std::string>(output_dir_str)->default_value("."));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, false))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  try {
    extract_stretches(config, std::filesystem::path(output_dir_str));
  } catch (std::exception& e) {
    std::cerr << "extract_stretches failed: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
