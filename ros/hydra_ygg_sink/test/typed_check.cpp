// Checks the typed Yggdrasil store on a saved Hydra graph and times it against the JSON path.
//
// usage: typed_check <dsg.json> [updates]
// Each update moves and relabels a few objects, deletes and adds places and edges, then the
// store copies the graph into Yggdrasil. Every update the graph read back out of Yggdrasil must
// equal the input exactly (same spark_dsg binary encoding of every node and edge).
#include "../src/yggdrasil_store.cpp"

#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <random>

using namespace hydra_ygg;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: typed_check <dsg.json> [updates]" << std::endl;
    return 2;
  }
  const size_t updates = argc > 2 ? std::stoul(argv[2]) : 20;
  auto graph = spark_dsg::DynamicSceneGraph::load(argv[1]);
  std::cout << "graph: " << graph->numNodes() << " nodes, " << graph->numEdges() << " edges"
            << std::endl;

  setenv("HYDRA_YGG_COMPARE_EVERY", "0", 1);
  auto store = hydra::GraphStore::factory()("both");
  std::mt19937 rng(7);
  bool ok = true;
  double typed_ms = 0, json_ms = 0, readback_ms = 0;
  auto json_mirror = new_mirror();
  NodeId next_place = spark_dsg::NodeSymbol('p', 900000);

  for (size_t u = 0; u <= updates; ++u) {
    if (u > 0) {
      // Hydra-style edits: in-place attribute changes, removed and new nodes and edges.
      std::vector<NodeId> objects, places;
      for (const auto& [id, node] : graph->getLayer(spark_dsg::DsgLayers::OBJECTS).nodes()) {
        objects.push_back(id);
      }
      for (const auto& [id, node] : graph->getLayer(spark_dsg::DsgLayers::PLACES).nodes()) {
        places.push_back(id);
      }
      for (int i = 0; i < 5 && !objects.empty(); ++i) {
        auto& attrs = graph->getNode(objects[rng() % objects.size()])
                          .attributes<spark_dsg::SemanticNodeAttributes>();
        attrs.position.x() += 0.01;
        attrs.semantic_label = rng() % 30;
      }
      for (int i = 0; i < 3 && places.size() > 3; ++i) {
        const size_t k = rng() % places.size();
        graph->removeNode(places[k]);
        places.erase(places.begin() + k);
      }
      for (int i = 0; i < 3; ++i) {
        auto attrs = std::make_unique<spark_dsg::PlaceNodeAttributes>();
        attrs->position << rng() % 10, rng() % 10, 1.0;
        graph->emplaceNode(spark_dsg::DsgLayers::PLACES, next_place, std::move(attrs));
        if (!places.empty()) {
          graph->insertEdge(next_place, places[rng() % places.size()]);
        }
        ++next_place;
      }
    }

    auto t = Clock::now();
    store->store(0, *graph);
    typed_ms += u ? msSince(t) : 0;

    t = Clock::now();
    json_mirror->apply_dsg_json(spark_dsg::io::json::writeGraph(*graph, false));
    json_ms += u ? msSince(t) : 0;

    t = Clock::now();
    const auto back = readBack(static_cast<YggdrasilStore&>(*store).typed());
    readback_ms += u ? msSince(t) : 0;
    const auto d = compare(*graph, *back);
    if (!d.empty()) {
      std::cout << "update " << u << ": DIFFERS " << d << std::endl;
      ok = false;
    }
  }

  const double n = updates;
  std::cout << "per update: typed copy " << typed_ms / n << " ms (JSON path " << json_ms / n
            << " ms), read back " << readback_ms / n << " ms" << std::endl;
  std::cout << "final graph: " << graph->numNodes() << " nodes, " << graph->numEdges()
            << " edges" << std::endl;
  std::cout << (ok ? "TYPED OK" : "TYPED FAILED") << std::endl;
  return ok ? 0 : 1;
}
