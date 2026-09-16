#!/usr/bin/env bash
# Native Ubuntu 26.04 installer. Run as your normal VM user, not via sudo bash.
set -euo pipefail
trap 'printf "Installation stopped at line %s. Fix the error above, then rerun this script.\n" "$LINENO" >&2' ERR

if [[ ! -r /etc/os-release ]]; then
  echo 'Run this installer inside the Ubuntu 26.04 VM.' >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 26.04 || "${VERSION_CODENAME:-}" != resolute ]]; then
  echo 'This installer targets Ubuntu 26.04 (Resolute) with ROS 2 Lyrical.' >&2
  echo "Detected: ${PRETTY_NAME:-unknown OS}. No packages were changed." >&2
  exit 1
fi
case "$(dpkg --print-architecture)" in
  arm64|amd64) ;;
  *) echo 'ROS 2 Lyrical binary installation requires arm64 or amd64.' >&2; exit 1 ;;
esac

sudo apt-get update
sudo apt-get install -y curl ca-certificates python3 locales software-properties-common
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
sudo add-apt-repository -y universe

# Separate download and parsing so a failed HTTP request cannot cause a
# misleading JSON error or a subsequent attempt to install a nonexistent file.
wafer_install_tmp=$(mktemp -d)
trap 'rm -rf -- "$wafer_install_tmp"' EXIT
curl --fail --show-error --silent --location --retry 3 \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  --output "$wafer_install_tmp/release.json"
wafer_ros_source_version=$(python3 - "$wafer_install_tmp/release.json" <<'PY'
import json
import re
import sys
from pathlib import Path
value = json.loads(Path(sys.argv[1]).read_text()).get('tag_name', '')
if not re.fullmatch(r'[A-Za-z0-9._-]+', value):
    raise SystemExit('ROS repository release response has no valid tag_name')
print(value)
PY
)
curl --fail --show-error --location --retry 3 \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${wafer_ros_source_version}/ros2-apt-source_${wafer_ros_source_version}.resolute_all.deb" \
  --output "$wafer_install_tmp/ros2-apt-source.deb"
sudo dpkg -i "$wafer_install_tmp/ros2-apt-source.deb"
sudo apt-get update
sudo apt-get install -y \
  ros-lyrical-desktop ros-lyrical-ros-gz ros-lyrical-cv-bridge \
  ros-lyrical-rqt-graph ros-lyrical-rqt-image-view \
  ros-dev-tools python3-pytest python3-opencv python3-numpy \
  python3-yaml python3-qrcode python3-pil python3-setuptools

# ROS setup scripts are not guaranteed to be nounset-safe.
set +u
source /opt/ros/lyrical/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro lyrical
printf '\nInstalled ROS 2 Lyrical and Gazebo Jetty for Ubuntu 26.04.\n'
printf 'Next, from the repository root:\n'
printf '  source /opt/ros/lyrical/setup.bash\n'
printf '  rosdep install --from-paths wafer_transport_sim --ignore-src -r -y --rosdistro lyrical\n'
printf '  colcon build --symlink-install --packages-select wafer_transport_sim\n'
printf '  source install/setup.bash\n'
printf '  ros2 launch wafer_transport_sim demo.launch.py\n'
