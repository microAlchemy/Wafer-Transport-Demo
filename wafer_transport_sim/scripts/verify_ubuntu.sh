#!/usr/bin/env bash
# Run from a built, sourced workspace on Ubuntu 24.04.
set -euo pipefail
command -v ros2 >/dev/null
command -v gz >/dev/null
share="$(ros2 pkg prefix --share wafer_transport_sim)"
export GZ_SIM_RESOURCE_PATH="$share/models:$share:${GZ_SIM_RESOURCE_PATH:-}"
gz sdf -k "$share/worlds/wafer_transport.sdf"
for model in "$share"/models/*/model.sdf; do
  gz sdf -k "$model"
done
for station in STATION_A STATION_B STATION_C; do
  ros2 run wafer_transport_sim integration_check --destination "$station"
done
for scenario in missing_images lost_line failed_pickup missing_target; do
  ros2 run wafer_transport_sim integration_check --scenario "$scenario"
done
