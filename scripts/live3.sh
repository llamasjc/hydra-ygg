#!/bin/bash
# Live 3-robot Hydra-Multi: station + one Hydra and one bag per robot, each robot in its own
# ROS domain. Run inside the hydra container. Logs and the station graph go to /tmp/live3.
#   bash live3.sh          start everything (Ctrl-C stops it all)

L=/tmp/live3; mkdir -p $L
BAGS=(/datasets/hydra_office_mcap /datasets/office06_bag /datasets/office12_bag)
source /root/hydra_ws/install/setup.bash
pkill -9 -f "ros2 bag play" ; pkill -9 -f hydra_ros_node ; pkill -f station_server ; sleep 1
trap 'kill 0' EXIT
cd /root/hydra_ws/src/hydra-yggdrasil
cargo build --release --bin station_server -q || exit 1
./target/release/station_server --robots 3 --align --deform --max-matching \
    --out $L/station.json --dsg-out $L/station_dsg.json --rooms --every 1 &
sleep 2
for N in 0 1 2; do
  ROS_DOMAIN_ID=$((10+N)) HYDRA_YGG_RECORD=tcp://127.0.0.1:$((7447+N)) HYDRA_GRAPH_BACKEND=yggdrasil \
    LD_PRELOAD=/root/hydra_ws/install/hydra_ygg_sink/lib/libhydra_ygg_sink.so \
    ros2 launch hydra_ros uhumans2.launch.yaml > $L/hydra$N.log 2>&1 &
done
sleep 15
for N in 0 1 2; do
  ROS_DOMAIN_ID=$((10+N)) ros2 bag play ${BAGS[$N]} --clock --rate 0.5 \
    --qos-profile-overrides-path /root/.tf_overrides.yaml > $L/bag$N.log 2>&1 &
done
echo "running; station reports below, logs in $L. Ctrl-C stops everything."
wait
