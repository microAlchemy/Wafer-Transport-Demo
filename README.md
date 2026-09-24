# Ten-glovebox U-layout wafer transport simulation

The default Gazebo world implements the supplied aerial U layout as a
Class 100 facility with **10 gloveboxes**: four across the top, two down the left,
and four across the bottom. Every glovebox is exactly
1.2192 × 0.9144 × 1.2192 m (4 × 3 × 4 ft), the route corridors are 0.3048 m (1 ft) wide, and the robot remains outside
every box.

A Yahboom Raspbot V2 follows a two-track black tape network. Track 1 is the main
lane farthest from the doors; Track 2 is the service lane beside them. At each
glovebox the route FSM stops at a pre-junction decision point, confirms that
glovebox's forward-facing AprilTag, turns onto Track 2 and stops outside the
entry guillotine door. Its onboard wand places the wafer on the entry handoff.
A separate orange rail robot inside that glovebox collects the wafer, lowers it
onto the blue process stage, holds it there for the configured processing
period, and carries it to the exit handoff. The Raspbot confirms the exit tag
plus the completed service cell, collects the wafer through the second door,
returns to Track 1 through the following rung, and repeats. After Glovebox 10 it
turns around at the end of Track 1 and returns home on Track 1 with the
processed wafer in its carrier.

No model is teleported. Raspbot and process-cell transfers use Gazebo detachable
joints, measured joint feedback, actual wafer pose, and supporting shelves.

## Run in the Ubuntu 26.04 UTM VM

The VM needs ROS 2 Lyrical and Gazebo Jetty. Stop any older launch with
**Ctrl-C**, share this project's `dist` folder in UTM, and run this whole block:

```bash
(
set -e
sudo mkdir -p /mnt/utm
if ! mountpoint -q /mnt/utm; then
  sudo mount -t 9p -o trans=virtio,version=9p2000.L,ro share /mnt/utm
fi
test -f /mnt/utm/wafer-transport-ubuntu26.04.zip
unzip -o /mnt/utm/wafer-transport-ubuntu26.04.zip -d ~/wafer-mobile-update
cd ~/wafer-mobile-update/Wafer-Transport-Demo
grep -qx 'ten-glovebox-u-tracks-20260924' wafer_transport_sim/config/release.txt || {
  echo 'STOP: UTM is seeing an older ZIP.'
  exit 1
}
source /opt/ros/lyrical/setup.bash
colcon --log-base log-u-tracks build \
  --base-paths ./wafer_transport_sim \
  --build-base build-u-tracks \
  --install-base install-u-tracks \
  --symlink-install \
  --packages-select wafer_transport_sim
source install-u-tracks/local_setup.bash
ros2 launch wafer_transport_sim demo.launch.py gui:=true
)
```

For later launches:

```bash
cd ~/wafer-mobile-update/Wafer-Transport-Demo
source /opt/ros/lyrical/setup.bash
source install-u-tracks/local_setup.bash
ros2 launch wafer_transport_sim demo.launch.py gui:=true
```

## Aerial layout

The 18 mm black tape follows the red perimeter route in the supplied sketch,
with two parallel U-shaped lanes and 20 crossing rungs:

- **Track 1** is the outer U, carrying the robot between boxes and back home.
- **Track 2** is the inner U beside the guillotine doors. Its centerline is
  0.3048 m from Track 1 on each straight leg.
- A rung before each box leads from Track 1 to Track 2; another after the
  box returns to Track 1. AprilTags authorize each choice in the FSM.
- Both tracks turn around the top-left and bottom-left corners. The robot
  slows and steers on its selected edge through those bends.

The tapes are black in Gazebo. The red in the sketch indicates their route; it
is not a physical red strip in the simulation. There is no third return lane.
The generated [overhead preview](docs/ten-glovebox-u-layout.png) shows the
committed box and tape positions.

The Raspbot chassis stays outside every glovebox. Only its telescoping wand
crosses an open door. Each box contains its own rail robot, blue process stage,
entry support, and exit support. The transparent walls and open roof keep the
complete handoff visible from the Gazebo aerial camera.

## Nodes in the default mission

`demo.launch.py` starts these application nodes:

| Node | Responsibility |
|---|---|
| `transport_controller` | Sole `/cmd_vel` owner; runs the two-track route, tag, junction, door, and Raspbot handoff state machine |
| `ir_line_follower` | Produces four IR probe readings and a steering candidate from the live robot pose and tape geometry |
| `apriltag_detector` | Decodes AprilTag 36h11 IDs from actual forward-camera frames and requires three consistent frames |
| `process_cell_controller` | Controls all 10 internal rail robots and verifies entry, process, and exit wafer placement |
| `parameter_bridge` | Bridges commands and feedback between ROS 2 and Gazebo |

Gazebo provides MecanumDrive, joint position control, joint feedback, pose
publishing, cameras, and detachable-joint systems.

## Mission sequence at every glovebox

```text
(Track 1) APPROACH_ENTRY          transit to the pre-junction decision stop
→ CONFIRM_ENTRY_TAG               fresh in-scope entry tag
→ ADVANCE_ENTRY_JUNCTION          creep to the junction mouth
→ ALIGN_ENTRY_JUNCTION            turn onto the selected rung
→ CROSS_ENTRY_JUNCTION            explicit selected-edge crossing
→ ALIGN_SERVICE_LANE
(Track 2) TRAVEL_TO_ENTRY
→ ALIGN_ENTRY
→ OPEN_ENTRY
→ carrier-to-entry handoff
→ CLOSE_ENTRY
→ internal robot collects wafer
→ blue-stage processing
→ internal robot places wafer at exit
→ TRAVEL_TO_EXIT
→ CONFIRM_EXIT_TAG                fresh exit tag plus the completed service cell
→ ALIGN_EXIT
→ OPEN_EXIT
→ exit-to-carrier handoff
→ CLOSE_EXIT
→ TRAVEL_TO_RETURN_JUNCTION
→ ALIGN_RETURN_JUNCTION
→ CROSS_RETURN_JUNCTION           back to Track 1
→ ALIGN_MAIN_LANE
```

The Raspbot remains stopped while a door moves or its wand is extended. It
continues only after joint feedback confirms that the door is closed and the
wand is stowed. The next glovebox cannot start until the preceding exit handoff
has been verified. After Glovebox 10 the FSM runs `RETURN_HOME`, `TURNAROUND`
(a measured 180° turn at the end of Track 1) and `HOME_RETURN`, and then
`VERIFY_DELIVERY`.

`transport_controller` also publishes two transient-local diagnostics on every
tick: `/transport/track` (`TRACK_1` or `TRACK_2`) and `/transport/route_segment`
(the selected edge, for example `main_04`, `rung_in_04`, `service_04`,
`rung_out_04`, `main_tail`, `main_home`).

## Internal process cells

Each glovebox contains one generated X/Z rail shuttle. Its vacuum cup parks a
derived lift stroke above every support surface, and the generated model and the
controller read the same heights from `four_room_layout.py`, so a trip is:
lower onto the supported wafer, attach and acknowledge, lift clear of the shelf,
translate to the blue stage, lower and release, verify the supported dwell for
`process_hold` seconds, pick the wafer back up, place it on the exit handoff and
retract. Gazebo attaches a detachable joint on its first update, so all ten
cells are parked on their own supports at startup: the transport may not start
its mission until `/process/cells_detached` reports true.

An active transfer faults on stale required feedback, on a motion or attachment
timeout, when the mobile vacuum or another cell still holds the wafer, and when
the wafer is no longer on the support that should hold it. A fault stops the
commanded joint motion, keeps the cell's attachment state and publishes
`/system/fault`, which stops the transport.

## AprilTags

Each of the 10 gloveboxes has an entry and an exit AprilTag 36h11 PNG in
`wafer_transport_sim/textures/room_N_{entry,exit}_apriltag.png`. IDs 0–19
map to `GLOVEBOX_01_ENTRY`, `GLOVEBOX_01_EXIT`, ..., `GLOVEBOX_10_EXIT`.

`wafer_transport_sim/scripts/generate_four_rooms.py` creates the 20 marker
images with OpenCV and places each image on a sign in the generated
`worlds/four_rooms.sdf`. `marker_pose()` in `four_room_layout.py` keeps signs
ahead of their track junctions and facing the Raspbot's forward camera.
`generate_four_rooms.py` can regenerate both the images and the world.

Gazebo's forward camera publishes `/camera/image_raw` through the bridge at
640 × 480 and 60 Hz. `apriltag_detector.py` uses OpenCV's 36h11 detector,
requires three consistent frames, and publishes the decoded room/side name on
`/detected_apriltag`. It also publishes `/apriltag/healthy`.

`four_room_controller.py` accepts only the expected tag for its current room
and entry/exit decision. It requires a fresh observation after the decision
window opens; wrong, stale and previously seen tags cannot authorize the
junction. The FSM then selects the Track 1 → Track 2 entry rung or the
Track 2 → Track 1 exit rung. IR sensors follow the selected black tape, while
odometry and model pose determine the actual stop. A tag alone never opens a
door or completes docking. The robot returns home on Track 1 after box 10.

These images and code serve the active mission; the retired QR station demo
and its textures are no longer installed.

## IR following

The four front IR probes are the primary steering input:

| Topic | Meaning |
|---|---|
| `/ir/values` | Four normalized reflectance values |
| `/ir/valid` | At least one probe sees tape |
| `/ir/cmd_vel` | Candidate forward/turn command |
| `/transport/steering_mode` | `IR` or `SIMULATED_IR` |

If the simulated IR process is absent, stale, off-tape, or produces an unusable
command, the controller follows the same generated tape from live Gazebo pose.
This explicit `SIMULATED_IR` mode is the requested “pretend sensor” recovery.
On each straight leg the IR candidate is the primary steering input; the
bounded junction states steer onto the edge the route FSM selected. The pose
fallback follows that same selected edge, so it can never choose a lane, and it
does not bypass door, attachment, odometry, pose, or AprilTag checks.

Force that recovery mode with:

```bash
ros2 launch wafer_transport_sim demo.launch.py ir_enabled:=false
```

## Useful topics

```bash
ros2 topic echo /transport/decision
ros2 topic echo /transport/state
ros2 topic echo /transport/steering_mode
ros2 topic echo /transport/track
ros2 topic echo /transport/route_segment
ros2 topic echo /detected_apriltag
ros2 topic echo /process/state
ros2 topic echo /process/ready
ros2 topic echo /process/cells_detached
ros2 topic echo /rooms/active
ros2 topic echo /rooms/completed
ros2 topic echo /system/fault
ros2 topic echo /wafer_delivered
ros2 topic info /cmd_vel --verbose
```

Door topics follow this pattern:

```text
/actuators/room_N_entry_z
/doors/room_N/entry/joint_states
/actuators/room_N_exit_z
/doors/room_N/exit/joint_states
```

Internal robots use:

```text
/actuators/room_N_process_x
/actuators/room_N_process_z
/process/room_N/joint_states
/attachments/process_N/{attach,detach,state}
```

## Configuration and verification

Mission parameters are in `wafer_transport_sim/config/four_rooms.yaml`. Geometry
and route constants are shared through `four_room_layout.py`; the historical
filename is retained so existing launch commands continue to work.

The two U-shaped tracks, the 20 transverse rungs, the selected-edge route plan and
the marker positions all come from the same module, and the world generator
draws the tape and the signs from those values. `four_room_check` records the
`/transport/track` and `/transport/route_segment` sequence on the VM and fails
the run if a Track 2 excursion is missing, out of order, or if the return home
ever leaves Track 1.

Portable checks:

```bash
PYTHONPATH=wafer_transport_sim python3 -m pytest wafer_transport_sim/test -q
python3 wafer_transport_sim/scripts/generate_four_rooms.py
python3 wafer_transport_sim/scripts/preview_two_track.py --output docs/ten-glovebox-u-layout.png
```

`generate_four_rooms.py` rebuilds the world, doors, process cells, textures and
bridge mapping from the shared constants, and `preview_two_track.py` draws the
top-view `docs/ten-glovebox-u-layout.png` from the world it
just wrote. The Ubuntu archive is packaged from a staging copy so no local
`dist`, cache or agent-work directory leaks into it:

```bash
stage=$(mktemp -d)
rsync -a --prune-empty-dirs --exclude '.git/' --exclude '.DS_Store' \
  --exclude '__pycache__/' --exclude '.pytest_cache/' --exclude '*.egg-info/' --exclude 'dist/' \
  --exclude 'docs/agent-work/' --exclude 'plot*.py' --exclude 'path*.png' \
  README.md LICENSE .gitignore docs wafer_transport_sim \
  "$stage/Wafer-Transport-Demo/"
(cd "$stage" && zip -qr -X wafer-transport-ubuntu26.04.zip Wafer-Transport-Demo)
mv "$stage/wafer-transport-ubuntu26.04.zip" dist/wafer-transport-ubuntu26.04.zip
```

Ubuntu integration checks:

```bash
ros2 run wafer_transport_sim four_room_check --gui
ros2 run wafer_transport_sim four_room_check --scenario missing_ir
ros2 run wafer_transport_sim four_room_check --scenario missing_apriltag
ros2 run wafer_transport_sim four_room_check --scenario failed_pickup
ros2 run wafer_transport_sim four_room_check --scenario failed_process
ros2 run wafer_transport_sim four_room_check --scenario stuck_door
```

The current macOS workspace has no ROS or Gazebo installation. Python state
machines, generated AprilTags, package metadata, and SDF/XML are checked locally;
contact dynamics, rendered tag visibility, 60 Hz camera performance, and the
complete physical mission must still be verified in the Ubuntu VM. The
`failed_process` run drops the `/attachments/process_1/attach` bridge so the
first internal cell never acknowledges its pickup, which must fault the cell and
stop the transport without a delivery.

Raspbot dimensions, model assumptions, and hardware limitations are documented
in [docs/raspbot_v2.md](docs/raspbot_v2.md). The package launches only the ten-glovebox AprilTag mission.

Junction arrival braking targets the stop position rather than the boundary of its tolerance window, avoiding a low-speed deadband stall before the tag decision. `/transport/decision` reports FSM state, expected tag, authorization, measured velocity and fault. Repeated tag logs are limited to one per ID per five simulation seconds.

Door clearance repair: handoff centres are 100 mm inside the front wall and the blue stage is 160 mm deep, keeping stationary supports clear of the closed panels. The closed-door check accepts the model’s downward lower-stop travel (-6 to +2 mm), retains a strict 2 mm upward-opening limit, and rejects stale or invalid feedback. Door faults include position and feedback age.

## Turn reliability

At each transverse tape connection the route FSM aligns in place, crosses the selected rung and aligns with the next lane. Steering aims beyond the end of a selected tape edge along its tangent so the target stays in front of the robot near the junction. Alignment uses a 0.12 rad/s minimum turn command outside a 0.03 rad junction heading tolerance (0.012 rad when facing a door); it then commands zero and waits for measured low odometry speed. These values are configured in `config/four_rooms.yaml`. `/transport/decision` includes `heading_error` while aligning for VM diagnosis.
