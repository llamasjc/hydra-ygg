// Links libygg_bridge.a from C++ the way the Hydra sink does, and checks a round trip.
// Usage: smoke [dsg.json]   (uses a small built-in graph when no file is given)
#include <fstream>
#include <iostream>
#include <sstream>

#include "rust/cxx.h"
#include "ygg-bridge/src/lib.rs.h"

static const char* kGraph = R"({"directed":false,"nodes":[
  {"id":1,"layer":3,"partition":0,"attributes":{"position":[1.0,2.0,3.0]}},
  {"id":2,"layer":3,"partition":0,"attributes":{"position":[0.0,0.0,0.0]}},
  {"id":3,"layer":4,"partition":0,"attributes":{"position":[0.5,0.5,0.5]}}],
  "edges":[{"source":1,"target":2,"info":{"weight":1.0}},
           {"source":3,"target":1,"info":{"weight":1.0}}]})";

int main(int argc, char** argv) {
  std::string json = kGraph;
  if (argc > 1) {
    std::stringstream ss;
    ss << std::ifstream(argv[1]).rdbuf();
    json = ss.str();
  }
  auto mirror = hydra_ygg::new_mirror();
  const auto stats = mirror->apply_dsg_json(json);
  const auto r = mirror->compare_with(json, 1e-5);
  std::cout << "nodes " << stats.nodes << ", nestings " << stats.nestings << ", import "
            << stats.import_us << " us, max pos err " << r.max_position_error << "\n";
  if (!r.equal) {
    std::cerr << "mirror differs from input:\n" << std::string(r.summary) << "\n";
    return 1;
  }
  try {
    mirror->apply_dsg_json("not json");
    std::cerr << "bad JSON was accepted\n";
    return 1;
  } catch (const ::rust::Error&) {
  }
  std::cout << "SMOKE OK\n";
  return 0;
}
