// Turns a station graph (replay --out station.json) into a Spark-DSG file that Hydra's
// static visualizer can show:
//
//   station_to_dsg station.json station_dsg.json
//   ros2 launch hydra_visualizer static_visualizer.launch.yaml scene_graph:=$PWD/station_dsg.json
//
// Each station node carries its robot's spark_dsg record (local id, robot-frame position). The
// records of one robot are read back with spark_dsg's binary reader, then every node is added
// under its station id with its station (transformed) position. Edges get default attributes.
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/serialization/binary_serialization.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
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

uint64_t localId(uint64_t global) {
  return (global >> 56) << 56 | (global & ((uint64_t(1) << 48) - 1));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: station_to_dsg <station.json> <out.json|out.sparkdsg>" << std::endl;
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
  for (const auto& e : station.at("edges")) {
    edges += graph->insertEdge(e.at("source").get<uint64_t>(), e.at("target").get<uint64_t>());
  }
  graph->save(argv[2], false);
  std::cout << "wrote " << argv[2] << ": " << graph->numNodes() << " nodes, " << edges
            << " edges" << std::endl;
}
