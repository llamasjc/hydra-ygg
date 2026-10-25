// Mirrors Hydra's published backend scene graph into Yggdrasil without touching Hydra.
//
// Subscribes (through hydra_ros's DsgReceiver) to `dsg`; remap it to /hydra/backend/dsg.
// Every received graph is written with Hydra's own JSON writer, rebuilt in Yggdrasil, and
// every `compare_every` updates the Yggdrasil export is diffed against Hydra's JSON.

#include <hydra_ros/utils/dsg_streaming_interface.h>
#include <ianvs/node_handle.h>

#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "rust/cxx.h"
#include "ygg_bridge_cxx/lib.h"

namespace {

class YggMirrorNode {
 public:
  explicit YggMirrorNode(rclcpp::Node& node)
      : node_(node),
        receiver_(ianvs::NodeHandle(node)),
        mirror_(hydra_ygg::new_mirror()),
        scratch_path_(node.declare_parameter<std::string>("scratch_path",
                                                          "/tmp/hydra_ygg_update.json")),
        output_path_(node.declare_parameter<std::string>("output_path", "/root/ygg_mirror.json")),
        compare_every_(node.declare_parameter<int>("compare_every", 10)),
        tolerance_(node.declare_parameter<double>("tolerance", 1e-5)) {
    timer_ = node.create_wall_timer(std::chrono::milliseconds(100), [this] { poll(); });
    std::cerr << "[YggMirror] listening on " << node.get_fully_qualified_name() << "/dsg"
              << std::endl;
  }

  ~YggMirrorNode() {
    try {
      mirror_->save(output_path_);
      std::cerr << "[YggMirror] saved mirror to " << output_path_ << " after " << updates_
                << " updates" << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[YggMirror] save failed: " << e.what() << std::endl;
    }
  }

 private:
  void poll() {
    if (!receiver_.updated() || !receiver_.graph()) {
      return;
    }
    receiver_.clearUpdated();
    try {
      receiver_.graph()->save(scratch_path_, false);
      std::stringstream ss;
      ss << std::ifstream(scratch_path_).rdbuf();
      const std::string json = ss.str();

      const auto stats = mirror_->apply_dsg_json(json);
      ++updates_;
      std::cerr << "[YggMirror] update " << updates_ << " nodes=" << stats.nodes
                << " nestings=" << stats.nestings << " import=" << stats.import_us / 1000
                << "ms" << std::endl;
      if (compare_every_ > 0 && updates_ % compare_every_ == 0) {
        const auto r = mirror_->compare_with(json, tolerance_);
        if (r.equal) {
          std::cerr << "[YggMirror] update " << updates_ << " matches Hydra (max pos err "
                    << r.max_position_error << ")" << std::endl;
        } else {
          std::cerr << "[YggMirror] update " << updates_ << " differs: nodes -"
                    << r.missing_nodes << "/+" << r.extra_nodes << " edges -" << r.missing_edges
                    << "/+" << r.extra_edges << " attrs " << r.mismatches << "\n"
                    << std::string(r.summary) << std::endl;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "[YggMirror] update failed: " << e.what() << std::endl;
    }
  }

  rclcpp::Node& node_;
  hydra::DsgReceiver receiver_;
  rust::Box<hydra_ygg::Mirror> mirror_;
  const std::string scratch_path_;
  const std::string output_path_;
  const int compare_every_;
  const double tolerance_;
  size_t updates_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ygg_mirror");
  {
    YggMirrorNode mirror(*node);
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
