// Turns a station graph (replay --out station.json) into a Spark-DSG file that Hydra's
// static visualizer can show:
//
//   station_to_dsg station.json station_dsg.json
//   ros2 launch hydra_visualizer static_visualizer.launch.yaml scene_graph:=$PWD/station_dsg.json
//
// Each station node carries its robot's spark_dsg record (local id, robot-frame position). The
// records of one robot are read back with spark_dsg's binary reader, then every node is added
// under its station id with its station (transformed) position. Edges get default attributes.
//
// With --rooms, the robots' rooms are dropped and rooms are detected again on the merged places
// with Hydra's RoomFinder, as Hydra-Multi does on its merged graph (uHumans2 settings):
//
//   station_to_dsg station.json station_dsg.json --rooms
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/serialization/binary_serialization.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <hydra/common/dsg_types.h>
#include <hydra/rooms/room_finder.h>

#include <fstream>
#include <cstring>
#include <iostream>
#include <memory>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using spark_dsg::DynamicSceneGraph;
using spark_dsg::serialization::BinarySerializer;

namespace {

std::vector<uint8_t> fromHex(const std::string& hex) {
  std::vector<uint8_t> out(hex.size() / 2);
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = static_cast<uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16));
  }
  return out;
}

//! A graph from a header and concatenated node records, as the Yggdrasil store reads back.
DynamicSceneGraph::Ptr readRecords(const std::vector<uint8_t>& header,
                                   const std::vector<uint8_t>& nodes) {
  std::vector<uint8_t> buffer = header;
  BinarySerializer s(&buffer);
  s.startDynamicArray();
  buffer.insert(buffer.end(), nodes.begin(), nodes.end());
  s.endDynamicArray();
  s.startDynamicArray();
  s.endDynamicArray();
  s.write(false);  // no mesh
  return spark_dsg::io::binary::readGraph(buffer);
}

//! Replace the rooms with rooms found on the places, as Hydra's UpdateRoomsFunctor does.
size_t detectRooms(DynamicSceneGraph& graph,
                   const std::vector<std::pair<uint64_t, uint64_t>>& foreign) {
  hydra::RoomFinderConfig config;  // uhumans2.yaml
  config.min_dilation_m = 0.5;
  config.max_dilation_m = 1.2;
  config.min_component_size = 10;
  config.min_room_size = 10;
  config.dilation_threshold_mode = hydra::DilationThresholdMode::PLATEAU;
  config.min_lifetime_length_m = 0.1;
  config.plateau_ratio = 0.15;
  config.clustering_mode = hydra::RoomClusterMode::NEIGHBORS;
  config.dilation_diff_threshold_m = -1.0;
  hydra::RoomFinder finder(config);

  const auto places = graph.findLayer(hydra::DsgLayers::PLACES);
  if (!places) {
    return 0;
  }
  // A later robot's place that the merge left unpaired but that sits on an earlier robot's
  // place is a duplicate; with its own edges it bridges rooms across doorways, so leave it out.
  constexpr double kDuplicate = 0.5;
  std::vector<std::pair<int, Eigen::Vector3d>> kept;
  for (const auto& [id, node] : places->nodes()) {
    if (spark_dsg::NodeSymbol(id).category() == 'p') {
      kept.emplace_back((id >> 48) & 0xff, node->attributes().position);
    }
  }
  auto clone = places->clone([&](const auto& node) {
    if (spark_dsg::NodeSymbol(node.id).category() != 'p') {
      return false;
    }
    const int robot = (node.id >> 48) & 0xff;
    const Eigen::Vector3d p = node.attributes().position;
    for (const auto& [other, q] : kept) {
      if (other < robot && (p - q).norm() < kDuplicate) {
        return false;
      }
    }
    return true;
  });
  // an edge drawn only by a robot that owns neither end would add a second, differently weighted
  // path through the owner's places (e.g. across a doorway); the owner's own edges decide
  for (const auto& [s, t] : foreign) {
    clone->removeEdge(s, t);
  }
  auto rooms = finder.findRooms(*clone);

  std::vector<spark_dsg::NodeId> old;
  for (const auto& [id, node] : graph.getLayer(hydra::DsgLayers::ROOMS).nodes()) {
    old.push_back(id);
  }
  for (const auto id : old) {
    graph.removeNode(id);
  }
  if (!rooms) {
    return 0;
  }
  for (auto&& [id, node] : rooms->nodes()) {
    graph.emplaceNode(hydra::DsgLayers::ROOMS, id, node->attributes().clone());
  }
  for (const auto& [key, edge] : rooms->edges()) {
    graph.insertEdge(edge.source, edge.target, edge.info->clone());
  }
  finder.addRoomPlaceEdges(graph);
  return rooms->numNodes();
}

uint64_t localId(uint64_t global) {
  return (global >> 56) << 56 | (global & ((uint64_t(1) << 48) - 1));
}

}  // namespace

int main(int argc, char** argv) {
  const bool rooms = argc == 4 && std::string(argv[3]) == "--rooms";
  if (argc != 3 && !rooms) {
    std::cerr << "usage: station_to_dsg <station.json> <out.json|out.sparkdsg> [--rooms]"
              << std::endl;
    return 2;
  }
  const auto station = nlohmann::json::parse(std::ifstream(argv[1]));
  const auto header = fromHex(station.at("record").get<std::string>());

  std::map<int, std::vector<uint8_t>> records;  // per robot
  for (const auto& n : station.at("nodes")) {
    const auto bytes = fromHex(n.at("record").get<std::string>());
    auto& r = records[(n.at("id").get<uint64_t>() >> 48) & 0xff];
    r.insert(r.end(), bytes.begin(), bytes.end());
  }
  std::map<int, DynamicSceneGraph::Ptr> robots;
  for (const auto& [robot, bytes] : records) {
    robots[robot] = readRecords(header, bytes);
  }

  auto graph = readRecords(header, {});
  for (const auto& n : station.at("nodes")) {
    const uint64_t id = n.at("id").get<uint64_t>();
    const auto& source = robots.at((id >> 48) & 0xff)->getNode(localId(id));
    auto attrs = source.attributes().clone();
    const auto& p = n.at("attributes").at("position");
    attrs->position << p[0].get<double>(), p[1].get<double>(), p[2].get<double>();
    graph->emplaceNode(source.layer, id, std::move(attrs));
  }
  size_t edges = 0;
  std::vector<std::pair<uint64_t, uint64_t>> foreign;
  for (const auto& e : station.at("edges")) {
    const uint64_t s = e.at("source").get<uint64_t>(), t = e.at("target").get<uint64_t>();
    if (e.contains("robots")) {
      bool owned = false;
      for (const auto& r : e.at("robots")) {
        const uint64_t robot = r.get<uint64_t>();
        owned = owned || robot == ((s >> 48) & 0xff) || robot == ((t >> 48) & 0xff);
      }
      if (!owned) {
        foreign.emplace_back(s, t);
      }
    }
    // the robot edge's own weight (for places: free space along the edge, which room detection
    // relies on), read from its spark_dsg record: ... <weighted: c2|c3> cb <weight: f64 LE>
    auto info = std::make_unique<spark_dsg::EdgeAttributes>();
    if (e.contains("info") && e.at("info").contains("record")) {
      const auto r = fromHex(e.at("info").at("record").get<std::string>());
      if (r.size() >= 10 && r[r.size() - 9] == 0xcb) {
        uint64_t bits = 0;
        for (size_t i = r.size(); i > r.size() - 8; --i) {
          bits = bits << 8 | r[i - 1];  // little-endian
        }
        double w;
        std::memcpy(&w, &bits, sizeof(w));
        info->weighted = r[r.size() - 10] == 0xc3;
        info->weight = w;
      }
    }
    edges += graph->insertEdge(s, t, std::move(info));
  }
  if (rooms) {
    std::cout << detectRooms(*graph, foreign) << " rooms detected on the merged places" << std::endl;
  }
  graph->save(argv[2], false);
  std::cout << "wrote " << argv[2] << ": " << graph->numNodes() << " nodes, " << edges
            << " edges" << std::endl;
}
