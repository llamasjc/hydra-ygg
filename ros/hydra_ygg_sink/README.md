# hydra_ygg_sink

Hydra backend sink that mirrors every backend update into Yggdrasil via the `ygg-bridge` crate.

## Build (inside the Hydra workspace / container)

```bash
# Rust toolchain needed in the container: curl https://sh.rustup.rs -sSf | sh -s -- -y
ln -s /path/to/hydra-ygg/ros/hydra_ygg_sink /mnt/storage/hydra_ws/src/hydra_ygg_sink
cd /mnt/storage/hydra_ws && colcon build --packages-select hydra_ygg_sink
```

## Enable

Add to the backend section of the Hydra config:

```yaml
backend:
  sinks:
    - type: YggdrasilSink
      output_path: /tmp/ygg_mirror.json
      compare_every: 10
```

The library must be loaded by Hydra. Add `hydra_ygg_sink` to the external libraries list
(`config_utilities` `external_libraries`) the same way `hydra_ros` is loaded.

Every `compare_every` updates the log shows `matches Hydra` or a diff summary.
