#!/usr/bin/env bash
# Run from a built, sourced workspace on Ubuntu 26.04 with ROS 2 Lyrical and Gazebo Jetty.
set -euo pipefail
if [[ "${ROS_DISTRO:-}" != lyrical ]]; then
  echo 'Source /opt/ros/lyrical/setup.bash and the built workspace first.' >&2
  exit 1
fi
command -v ros2 >/dev/null
command -v gz >/dev/null
gz sim --versions
python3 -c 'import rclpy, cv2, cv_bridge; print("ROS/OpenCV imports OK; OpenCV", cv2.__version__)'
share="$(ros2 pkg prefix --share wafer_transport_sim)"
export GZ_SIM_RESOURCE_PATH="$share/models:$share:${GZ_SIM_RESOURCE_PATH:-}"
gz sdf -k "$share/worlds/wafer_transport.sdf"
for model in "$share"/models/*/model.sdf; do
  gz sdf -k "$model"
done
for station in STATION_A STATION_B STATION_C; do
  ros2 run wafer_transport_sim integration_check --destination "$station"
done
for scenario in missing_images lost_line failed_pickup missing_target stuck_door room_lost_line room_missing_images; do
  ros2 run wafer_transport_sim integration_check --scenario "$scenario"
done
