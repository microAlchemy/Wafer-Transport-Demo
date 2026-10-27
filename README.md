# Raspbot V2 chassis starter

This is a minimal ROS 2/Gazebo package named `raspbot_chassis`. It places the Raspbot V2 chassis and four-wheel drive model on a plain floor. It starts no controller and has no tape, path, PID, AprilTags, or gloveboxes. The robot stays still until you publish a velocity command.

## Run in Ubuntu 26.04

Use ROS 2 Lyrical and Gazebo Jetty. If they are missing, run `bash raspbot_chassis/scripts/install_ubuntu.sh` once from the repository root.

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths raspbot_chassis --ignore-src -r -y --rosdistro lyrical
colcon build --symlink-install --packages-select raspbot_chassis
source install/local_setup.bash
ros2 launch raspbot_chassis chassis.launch.py
```

The new package and launch names prevent ROS from selecting an older `wafer_transport_sim` installation. You can confirm which package ROS found with `ros2 pkg prefix --share raspbot_chassis`.

## Transfer with UTM

In UTM's VM settings, set the shared directory to
`/Users/srigan/Wafer-Transport-Demo/dist`. The ZIP must be visible inside the VM
before building. In a fresh Ubuntu terminal, run:

```bash
sudo mkdir -p /mnt/utm
if ! mountpoint -q /mnt/utm; then
  sudo mount -t 9p -o trans=virtio,version=9p2000.L,ro share /mnt/utm
fi
ls -lh /mnt/utm/raspbot-chassis-v2-ubuntu26.04.zip
mkdir -p ~/chassis-v2
unzip -o /mnt/utm/raspbot-chassis-v2-ubuntu26.04.zip -d ~/chassis-v2
cd ~/chassis-v2/Wafer-Transport-Demo
```

Then run the build and launch commands above. Close any previous Gazebo demo before starting this one.
If `ls` says the ZIP is missing, check `find /mnt/utm -maxdepth 3 -name 'raspbot-chassis-v2-ubuntu26.04.zip'`.
If that finds nothing, the VM is sharing a different Mac folder; change UTM's shared directory and remount it.

## Interfaces for your controller

| ROS topic | Type | Direction | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | publish | `linear.x` drives forward; `angular.z` turns. |
| `/odom` | `nav_msgs/msg/Odometry` | subscribe | Read robot motion. |
| `/clock` | `rosgraph_msgs/msg/Clock` | subscribe | Use simulation time. |

The model keeps the original Raspbot V2 chassis, wheel joints, wheel positions, and MecanumDrive geometry. Add your own ROS node for PID and path following.
