#pragma once
#include <hydra/backend/backend_module.h>

#include <memory>
#include <string>

namespace hydra_ygg {

struct Mirror;

// Mirrors every backend update into a Yggdrasil scene graph (stage A: full rebuild per update).
class YggdrasilSink : public hydra::BackendModule::Sink {
 public:
  struct Config {
    std::string scratch_path = "/tmp/hydra_ygg_update.json";
    std::string output_path = "";  // Yggdrasil graph saved here at shutdown if set.
    int compare_every = 10;        // Diff mirror against Hydra every N updates (0 = never).
    double tolerance = 1e-5;
  } const config;

  explicit YggdrasilSink(const Config& config);
  ~YggdrasilSink() override;

  void call(uint64_t timestamp_ns,
            const spark_dsg::DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) const override;
  std::string printInfo() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

void declare_config(YggdrasilSink::Config& config);

}  // namespace hydra_ygg
