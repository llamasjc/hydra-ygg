# hydra_ygg_sink

Library preloaded into Hydra that puts its scene graph store and nearest-node search on
Yggdrasil (`YggdrasilStore` in `src/yggdrasil_store.cpp`, via the `ygg-bridge` crate).

## Build (inside the Hydra workspace / container)

```bash
cd /root/hydra_ws && colcon build --packages-select hydra_ygg_sink --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Enable

No Hydra config entry is needed (the old `YggdrasilSink` sink is gone; remove it from older
configs). Preload the library and pick backends with environment variables:

```bash
LD_PRELOAD=/root/hydra_ws/install/hydra_ygg_sink/lib/libhydra_ygg_sink.so \
SPARK_DSG_BACKEND=both HYDRA_GRAPH_BACKEND=both HYDRA_NN_BACKEND=both \
ros2 launch hydra_ros uhumans2.launch.yaml
```

`both` runs spark_dsg and Yggdrasil side by side and every `HYDRA_YGG_COMPARE_EVERY` (default
10) updates logs `matches spark_dsg` or a diff; `yggdrasil` publishes the graph read back
from Yggdrasil. `HYDRA_YGG_THREADS` sets the encoding threads (default up to 8).
