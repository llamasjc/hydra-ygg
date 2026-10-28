// Runs spark_dsg with Yggdrasil as its structure store (SPARK_DSG_BACKEND=both) on a real
// graph, editing it the ways Hydra's backend does, and asks every structure question after
// each step. Any disagreement between Yggdrasil and spark_dsg is counted by spark_dsg.
//
//   SPARK_DSG_BACKEND=both LD_PRELOAD=libhydra_ygg_sink.so backend_check <dsg.json>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/graph_backend.h>

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using namespace spark_dsg;

size_t askEverything(const DynamicSceneGraph& graph, const DynamicSceneGraph& reference) {
  size_t asked = 2;
  graph.numNodes();
  graph.numEdges();
  // every node and edge of the reference, whether or not this graph still has it
  for (const auto& [lid, layer] : reference.layers()) {
    for (const auto& [id, node] : layer->nodes()) {
      graph.hasNode(id);
      ++asked;
      for (const auto sibling : node->siblings()) {
        graph.hasEdge(id, sibling);
        ++asked;
      }
      for (const auto parent : node->parents()) {
        graph.hasEdge(id, parent);
        graph.hasEdge(parent, id);
        asked += 2;
      }
    }
  }
  return asked;
}

int main(int argc, char** argv) {
  if (argc < 2 || GraphBackend::mode() != GraphBackend::Mode::BOTH) {
    std::cerr << "usage: SPARK_DSG_BACKEND=both backend_check <dsg.json>" << std::endl;
    return 2;
  }
  const auto source = DynamicSceneGraph::load(argv[1]);
  DynamicSceneGraph graph;
  size_t asked = 0;
  auto step = [&](const std::string& what) {
    asked += askEverything(graph, *source);
    std::cout << what << ": nodes=" << graph.numNodes() << " edges=" << graph.numEdges()
              << " disagreements so far=" << GraphBackend::mismatches() << std::endl;
  };

  const auto start = std::chrono::steady_clock::now();
  graph.mergeGraph(*source);
  step("mergeGraph");

  std::mt19937 rng(7);
  std::vector<NodeId> ids;
  for (const auto& [lid, layer] : graph.layers()) {
    for (const auto& [id, node] : layer->nodes()) ids.push_back(id);
  }
  std::shuffle(ids.begin(), ids.end(), rng);
  for (size_t i = 0; i < 100 && i < ids.size(); ++i) graph.removeNode(ids[i]);
  step("removeNode x100");

  for (const auto& [lid, layer] : graph.layers()) {
    std::vector<NodeId> layer_ids;
    for (const auto& [id, node] : layer->nodes()) layer_ids.push_back(id);
    for (size_t i = 0; i + 1 < layer_ids.size() && i < 40; i += 2) {
      graph.mergeNodes(layer_ids[i], layer_ids[i + 1]);
    }
  }
  step("mergeNodes");

  std::vector<std::pair<NodeId, NodeId>> edges;
  for (const auto& [lid, layer] : graph.layers()) {
    for (const auto& [key, edge] : layer->edges()) edges.emplace_back(edge.source, edge.target);
  }
  for (size_t i = 0; i < edges.size(); i += 7) graph.removeEdge(edges[i].first, edges[i].second);
  step("removeEdge");

  graph.mergeGraph(*source);
  step("mergeGraph again");

  graph.clear();
  graph.mergeGraph(*source);
  step("clear + mergeGraph");

  const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start);
  std::cout << asked << " structure queries, " << GraphBackend::mismatches()
            << " disagreements, " << ms.count() << " ms" << std::endl;
  const bool ok = GraphBackend::mismatches() == 0;
  std::cout << (ok ? "BACKEND OK" : "BACKEND FAILED") << std::endl;
  return ok ? 0 : 1;
}
