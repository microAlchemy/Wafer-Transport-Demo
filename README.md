# Four-room wafer transport simulation

The default demonstration follows the supplied four-room layout: two Class 100
rooms above a rounded rectangular route and two below. A mobile robot follows
continuous **black floor tape through all four rooms**. Each room has separate,
visible guillotine entry and exit doors. The vacuum wand is mounted on the robot;
there is no stationary gantry.

The automatic mission runs once, clockwise:

1. **Room 1, top left:** enter, close the entry door, pick the wafer up from its
   process table, and place it in the onboard carrier.
2. **Room 2, top right:** deposit the wafer on the process table, verify support,
   then pick it back up and load the carrier.
3. **Room 3, bottom right:** repeat the table-to-carrier transfer.
4. **Room 4, bottom left:** leave the wafer on the destination table.
5. Exit room 4, close its door, and follow the left bend back to the starting
   position **outside all rooms**. Report `TRANSPORT COMPLETE` only after checking
   the final wafer placement and stopped robot.

The intermediate rooms demonstrate handoffs, not material processing. All wafer
motion uses the robot's physical wand joints and acknowledged vacuum attachment;
there are no object pose-setting calls. The carrier stays on the robot throughout.

**Validation:** local Python, XML/resource, QR image, camera-projection, controller,
and kinematic tests run on macOS. The new four-room world has **not been run in
Gazebo here**: this Mac has no ROS/Gazebo installation. In particular, door and
payload dynamics, contact behavior when picking back up from the carrier, and
rendered QR visibility still need the Ubuntu integration checks below. A passing
idealized controller test is not a successful physical simulation.

## Run on the existing UTM Ubuntu VM

Requires Ubuntu 26.04 ARM64 or amd64, ROS 2 Lyrical and Gazebo Jetty. Keep the
existing installation. On a new Ubuntu installation, run
`bash wafer_transport_sim/scripts/install_ubuntu.sh` first; dependencies and
manual installation commands are in [the installation guide](docs/single_room.md#requirements-and-installation).

Stop any running demonstration with **Ctrl-C**. Share the Mac project's `dist`
folder in UTM, then run this **whole block in Ubuntu**:

```bash
(
set -e
sudo mkdir -p /mnt/utm
if ! mountpoint -q /mnt/utm; then
  sudo mount -t 9p -o trans=virtio,version=9p2000.L,ro share /mnt/utm
fi
test -f /mnt/utm/wafer-transport-ubuntu26.04.zip || {
  echo "STOP: Share the Mac project's dist folder in UTM."
  exit 1
}
unzip -o /mnt/utm/wafer-transport-ubuntu26.04.zip -d ~/wafer-mobile-update
cd ~/wafer-mobile-update/Wafer-Transport-Demo
test -f wafer_transport_sim/worlds/four_rooms.sdf || {
  echo 'STOP: The shared ZIP is still the single-room version.'
  exit 1
}
source /opt/ros/lyrical/setup.bash
colcon build --base-paths ./wafer_transport_sim --symlink-install --packages-select wafer_transport_sim
source install/local_setup.bash
ros2 launch wafer_transport_sim demo.launch.py layout:=four_rooms
)
```

For later launches, from the updated project directory:

```bash
source /opt/ros/lyrical/setup.bash
source install/local_setup.bash
ros2 launch wafer_transport_sim demo.launch.py
```

Options:

```bash
ros2 launch wafer_transport_sim demo.launch.py gui:=false
ros2 launch wafer_transport_sim demo.launch.py config:=/absolute/path/four_rooms.yaml
# The previous single-room/station demonstration remains available explicitly:
ros2 launch wafer_transport_sim demo.launch.py layout:=single_room destination_station:=STATION_C
```

Headless operation still needs graphics support for the cameras. If the shared
ZIP disappears after reboot, remount it using the block above. Build from the
project directory with `--base-paths`, not from `~`, where old copies cause duplicate
package errors. Existing home-directory ZIPs may be stale.

## Layout and motion

| Room | Position in image | Tape stop (world X, Y), m | Transfer |
|---|---|---|---|
| ROOM_1 | Top left | −0.95, +0.75 | Table → carrier |
| ROOM_2 | Top right | +0.95, +0.75 | Carrier → table → carrier |
| ROOM_3 | Bottom right | +0.95, −0.75 | Carrier → table → carrier |
| ROOM_4 | Bottom left | −0.95, −0.75 | Carrier → table |

The floor is 4.8 × 3.2 m. Each room is 1.0 × 0.95 m and 1.10 m high, with
transparent walls and an open roof for viewing. The tape is 18 mm wide; the bends
have a 0.50 m radius. The robot starts at (−1.70, +0.75), facing right. These are
new demonstration dimensions chosen to fit four rooms, not the old single-room
footprint or dimensions inferred from the reference image.

The robot travels through each room horizontally. At the interior stop it turns
90 degrees toward the process table, extends its onboard wand 0.25 m, performs the
transfer, retracts, then turns back toward the exit. The entry door closes only
after the robot is inside; the exit opens only after the wand is stowed. The exit
door closes only after the entire robot/wand envelope is clear and odometry
reports low velocity. All eight doors have separate commands and measured joints.

The configured speeds are 0.08 m/s around the loop and 0.06 m/s through rooms;
steering, stopping, and turns reduce effective speed. Door targets ramp at
0.10 m/s over a 0.52 m stroke. VM rendering can make wall-clock time considerably
longer than simulation time. The camera controls forward steering; model pose
sets route progress and stop/clearance checks, while odometry verifies the robot
has stopped. Four-room docking does **not** reuse the old straight-corridor
odometry coordinates.

## Controls, states, and diagnostics

`demo.launch.py` selects the matching world, YAML, and explicit-direction bridges.
In four-room mode it launches `four_room_controller` as the node
`/transport_controller`, plus `line_follower` and `qr_detector`. The controller
combines room sequencing, wand control, and command arbitration so there is
exactly one `/cmd_vel` publisher. The legacy handler, manager, and docking nodes
are only started in `layout:=single_room`.

Models are loaded once through local SDF includes: one robot, carrier, and wafer,
plus eight independently controlled doors. The room walls and tape are static
world geometry. Assets install into the package share directory; source checkout
paths are not required at launch.

A typical room sequence is:

```text
APPROACH_ENTRY → OPEN_ENTRY → ENTER_ROOM → CONFIRM_ROOM → CLOSE_ENTRY
→ TURN_TO_TABLE
→ PICK_POSITION → PICK_LOWER → ATTACH → LIFT
→ PLACE_POSITION → PLACE_LOWER → RELEASE → RETRACT → VERIFY_PLACE
→ TURN_TO_ROUTE → OPEN_EXIT → EXIT_ROOM → CLOSE_EXIT
```

Rooms 2 and 3 run the transfer sequence twice. Room 4 is followed by
`RETURN_HOME → VERIFY_DELIVERY → COMPLETE`. Console output identifies the active
room and each table/carrier handoff. `VERIFY_PLACE` needs acknowledged detachment,
a stowed wand, and fresh wafer pose on the receiving support for at least one
simulation second. Waiting alone never substitutes for successful motion.

Each room has a real `ROOM_1` … `ROOM_4` QR texture. Three consistent decoded
camera frames authorize that room's transfer. Wrong room IDs are ignored; reaching
the expected coordinates alone cannot authorize wafer handling. If a required
image or pose goes stale, tape is lost during forward travel, attachment fails,
a door is not open during passage, or a state times out, the controller enters
`FAULT`, sends zero velocity, and holds the current joint positions. Restart the
launch after fixing the problem; in-place world reset is not supported.

| Topic | Purpose |
|---|---|
| `/transport/state`, `/system/state` | Mission state, reliable/transient-local |
| `/rooms/active`, `/rooms/completed` | Active room and ordered verified handoffs |
| `/system/fault` | Fault reason, reliable/transient-local |
| `/wafer_delivered` | True only after final placement and return outside |
| `/carrier_ready` | Wafer carried onboard; false after final unloading |
| `/detected_station` | Camera-decoded ROOM IDs in this layout |
| `/line/cmd_vel` | Camera steering candidate; never drives wheels directly |
| `/cmd_vel` | Sole authoritative wheel command |
| `/docking_distance` | Remaining tape distance to the current route stop |
| `/doors/room_N/entry/joint_states` | Entry door feedback (also `exit`) |
| `/actuators/room_N_entry_z` | Door position target (also `exit`) |
| `/attachments/vacuum/state`, `/attachments/carrier/state` | Gazebo attachment acknowledgement |
| `/poses/wafer`, `/poses/carrier`, `/poses/transport_robot` | Actual model poses |

`/carrier_delivered` belongs to the legacy carrier-deposition mission; the new
mission delivers the **wafer**, so observe `/wafer_delivered`.

Useful commands in a second sourced terminal:

```bash
ros2 topic echo /transport/state
ros2 topic echo /system/fault
ros2 topic echo /rooms/completed
ros2 topic echo /wafer_delivered
ros2 topic echo /detected_station
ros2 topic echo /doors/room_1/entry/joint_states
ros2 run rqt_image_view rqt_image_view /down_camera/image_raw
ros2 topic info /cmd_vel --verbose
```

Edit `config/four_rooms.yaml` before launch to change speeds, timeouts, or
`intermediate_transfers` (false passes through rooms 2 and 3 without unloading).
Room order/geometry are structural constants in `four_room_layout.py`, shared by
the generator and controller. `destination_station` is a legacy-only option and
is rejected in four-room mode to avoid silently running an unintended route.

## Verification and regeneration

Local checks from the project root:

```bash
PYTHONPATH=wafer_transport_sim python3 -m pytest wafer_transport_sim/test -q
python3 wafer_transport_sim/scripts/generate_assets.py
python3 wafer_transport_sim/scripts/generate_four_rooms.py
```

On Ubuntu, after building and sourcing, stop any other simulation and run:

```bash
# Runs Gazebo, observes real camera IDs and supported placements, and saves logs/JSON.
ros2 run wafer_transport_sim four_room_check --gui
# Headless nominal mission:
ros2 run wafer_transport_sim four_room_check
# Fault scenarios, one at a time:
ros2 run wafer_transport_sim four_room_check --scenario lost_line
ros2 run wafer_transport_sim four_room_check --scenario missing_images
ros2 run wafer_transport_sim four_room_check --scenario missing_room_qr
ros2 run wafer_transport_sim four_room_check --scenario failed_pickup
ros2 run wafer_transport_sim four_room_check --scenario stuck_door
```

Reports go to `log/four_rooms/`. The runner checks the ordered room handoffs,
actual QR observations, wafer support at every deposit, closed doors, final wafer
pose, and robot return outside. It observes for two more simulation seconds after
completion. Fault tests require the injection stage to have been reached, a
reported fault, zero velocity, and no false delivery flag. The default wall-clock
limit is 1800 seconds and can be changed with `--timeout` for a slow VM.

`bash wafer_transport_sim/scripts/verify_ubuntu.sh` validates SDF with Gazebo's
parser, then runs both four-room and legacy integration suites. These runtime
checks are provided but **have not been executed on this Mac**. The idealized
vacuum, 100 mm demonstration wafer, unmodelled contamination/processing, and
pose-based route localization remain simulation simplifications.

## Project files

```text
wafer_transport_sim/
  launch/demo.launch.py
  config/{four_rooms.yaml,four_rooms_bridge.yaml,simulation.yaml,bridge.yaml}
  worlds/{four_rooms.sdf,wafer_transport.sdf}
  models/{transport_robot,carrier,wafer,room_1_entry_door,...,room_4_exit_door,...}/
  textures/{room_1_qr.png,...,room_4_qr.png,...}
  scripts/{generate_assets.py,generate_four_rooms.py,install_ubuntu.sh,verify_ubuntu.sh}
  wafer_transport_sim/{four_room_layout.py,four_room_controller.py,four_room_check.py,...}
  test/{test_four_rooms.py,test_controllers.py,test_assets.py,test_vision.py,...}
```

The previous single-room architecture and station workflow are documented in
[docs/single_room.md](docs/single_room.md).
