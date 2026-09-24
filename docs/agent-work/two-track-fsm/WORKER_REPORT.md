# Worker report: two black tracks with AprilTag/FSM junction decisions

STATUS: ready_for_review

Task: `/root/two_track_fsm` (implementation worker, DeepSeek V4.1 Flash).
Workspace: `/Users/srigan/Wafer-Transport-Demo` (shared tree; the dirty baseline
captured at `/tmp/wafer-two-track-baseline-20260923` was preserved).
Release marker: `eleven-glovebox-two-track-fsm-20260923`.

## Changed paths and behaviour

- `wafer_transport_sim/wafer_transport_sim/four_room_layout.py` - replaced the
  closed U-detour loop with a finite two-track network. Track 1 (`TRACK_1_Y`,
  the published `MAIN_Y`) runs from `START[0]` to `TURN_X`; Track 2
  (`TRACK_2_Y`, the published corridor lane) runs beside the doors; 22 straight
  transverse rungs sit at `room.x ± 0.45 m`. New `Edge` type, `EDGES`,
  `edge_project`/`edge_point`/`edge_heading`, `tape_distance` over
  `TAPE_SEGMENTS`, `main_edge`/`service_edge`/`entry_rung`/`exit_rung`,
  `entry_decision_stop`, `room_stop` on the service edge, `marker_pose`,
  `tag_decision_stop`, `route_plan` (46 edges) and `MISSION_LENGTH` (41.024 m).
  No third return lane, no per-door bump, `RETURN_Y`/`RADIUS`/`project`/
  `point_at`/`heading_at`/`LENGTH` removed.
- `wafer_transport_sim/wafer_transport_sim/four_room_controller.py` - FSM now
  selects one finite edge and drives measured along-edge targets:
  `APPROACH_ENTRY → CONFIRM_ENTRY_TAG → ADVANCE_ENTRY_JUNCTION →
  ALIGN_ENTRY_JUNCTION → CROSS_ENTRY_JUNCTION → ALIGN_SERVICE_LANE →
  TRAVEL_TO_ENTRY → ALIGN_ENTRY → OPEN_ENTRY → transfer → CLOSE_ENTRY →
  ALIGN_AFTER_ENTRY → TRAVEL_TO_EXIT → CONFIRM_EXIT_TAG → ALIGN_EXIT →
  OPEN_EXIT → transfer → CLOSE_EXIT → ALIGN_AFTER_EXIT →
  TRAVEL_TO_RETURN_JUNCTION → ALIGN_RETURN_JUNCTION → CROSS_RETURN_JUNCTION →
  ALIGN_MAIN_LANE`, then `RETURN_HOME → TURNAROUND (measured 180°) →
  HOME_RETURN → VERIFY_DELIVERY`. `follow()` projects only onto the selected
  edge, faults on deviation/backwards jumps, steers with the IR candidate on
  straights and with the explicit lookahead in junction states; `SIMULATED_IR`
  follows the same selected edge. The last probe length of any leg
  (`PROBE_LOOKAHEAD`, 0.11 m) also steers the selected edge, because a stop is
  exactly where the tape ends under the nose. Tag authorisation is a bounded
  `(room, decision, since)` scope that ignores wrong-room, out-of-scope and
  stale observations and is consumed at the junction. New transient-local
  `/transport/track` and `/transport/route_segment`, plus a bounded
  exit-decision timeout when the service cell never delivers. Doors, transfers,
  attachment and wand interlocks are unchanged.
- `wafer_transport_sim/wafer_transport_sim/ir_line_follower.py` - probes now
  sample every physical tape segment (`tape_distance`) instead of one loop.
- `wafer_transport_sim/scripts/generate_four_rooms.py` - draws the two lanes and
  22 rungs from `TAPE_SEGMENTS` and places both AprilTags from `marker_pose`
  (0.25 m beyond the guarded junction, facing the approach lane).
- `wafer_transport_sim/scripts/preview_two_track.py` (new) - static overhead
  preview rendered from the generated `worlds/four_rooms.sdf`.
- `wafer_transport_sim/wafer_transport_sim/four_room_check.py` - observes
  `/transport/track` + `/transport/route_segment` and fails the VM run if a
  Track 2 excursion is missing/out of order or if the return home leaves Track 1.
- `wafer_transport_sim/config/four_rooms.yaml` - `mission_timeout` 900 → 1800 s
  (the two-track route is longer; the controller default follows).
- `wafer_transport_sim/config/release.txt`, `README.md` - new marker and
  two-track documentation (task, topics, tag scoping, packaging command).
- `wafer_transport_sim/test/test_four_rooms.py`, `test/test_raspbot.py` -
  reworked/added tests (below).
- `wafer_transport_sim/worlds/four_rooms.sdf` (24 tape visuals),
  `wafer_transport_sim/textures/*_apriltag.png` (unchanged tag IDs) and the
  regenerated models are the generator output.
- `docs/agent-work/two-track-fsm/two-track-overhead.png` - overhead preview of
  the committed geometry for root review.
- `dist/wafer-transport-ubuntu26.04.zip` - rebuilt release archive.

## Verification (commands, exit status, results)

1. `cd wafer_transport_sim && PYTHONPATH=. /usr/bin/python3 -m pytest test -q`
   - exit 0, **203 passed** (baseline before this task: 162 passed).
   Focused evidence in that suite:
   - `test_two_track_tape_network_geometry`, `test_geometry_keeps_robot_lane_and_`
     `door_clearance` - two continuous lanes 0.3048 m apart, 22 transverse
     rungs, no third lane, lane/box/door clearances (turning envelope 0.1333 m;
     22.8 mm to the door posts, 19.1 mm to the box front plane, 0.2207 m to the
     nearer box corner).
   - `test_world_tape_and_markers_match_the_shared_geometry`,
     `test_overhead_preview_draws_two_lanes_and_transverse_junctions` - the
     committed world and the preview show 2 lanes + 22 junctions and nothing
     above the service lane.
   - `test_marker_is_visible_from_the_pre_junction_decision_stop` - projects the
     generated marker through the generated camera (mount 0.170 m forward,
     0.242 m up, 110° HFOV, 640×480, 224.1 px focal): entry marker 0.280 m deep =
     160 px side, exit marker 0.310 m = 145 px, all corners inside the frame.
   - `test_real_apriltag_textures_decode` - all 22 committed textures still
     decode to their original 36h11 IDs.
   - `test_full_eleven_glovebox_two_track_mission`,
     `test_return_home_stays_on_track_1_without_projection_jumps` - full
     mission, per-room state order, Track 1 → Track 2 → Track 1, reverse
     main-lane return with monotone along-edge travel.
   - `test_wrong_room_tags_are_ignored_at_the_decision`,
     `test_tag_observed_before_the_decision_scope_does_not_authorise`,
     `test_same_tag_observed_in_a_different_decision_state_is_ignored`,
     `test_authorisation_is_one_shot_and_room_scoped`,
     `test_missing_decision_tag_faults_before_the_junction`,
     `test_exit_decision_requires_the_completed_service_cell`,
     `test_service_tags_never_pull_the_return_home_off_track_1`,
     `test_junction_states_follow_the_selected_edge_not_the_ir_candidate`,
     `test_pose_fallback_steers_the_selected_edge_but_never_authorises_tags`,
     `test_mission_completes_on_ir_alone_with_the_pose_fallback_disabled` - tag
     scoping, one-shot authorisation, selected-edge junction control and IR
     primacy on the straights.
   - door/fault tests keep zero-motion-during-shot and no-delivery guarantees.
2. `/usr/bin/python3 wafer_transport_sim/scripts/generate_four_rooms.py` -
   exit 0, regenerates world/config/models/textures; a second run changes no
   file (idempotent). `config/four_rooms_bridge.yaml` and every
   `models/**/model.sdf` are byte-identical to the preserved baseline.
3. `/usr/bin/python3 wafer_transport_sim/scripts/preview_two_track.py` -
   exit 0, wrote `docs/agent-work/two-track-fsm/two-track-overhead.png`
   (1517×199) from the generated tape rectangles.
4. Release archive built from the final tree with
   `rsync -a --prune-empty-dirs --exclude ... README.md LICENSE .gitignore docs
   wafer_transport_sim <stage>/Wafer-Transport-Demo/` then
   `zip -qr -X dist/wafer-transport-ubuntu26.04.zip Wafer-Transport-Demo`
   (excludes `.git`, `dist`, `docs/agent-work`, caches, `plot*.py`, `path*.png`).
   - `unzip -l dist/wafer-transport-ubuntu26.04.zip | tail -3` - 237 files.
   - Extracted-archive checks: `config/release.txt` =
     `eleven-glovebox-two-track-fsm-20260923`, marker present once in `README.md`,
     no `docs/agent-work`, 24 `tape_*` visuals in the packaged world,
     `preview_two_track.py` present.
   - `cd <extract>/Wafer-Transport-Demo/wafer_transport_sim && PYTHONPATH=.
     /usr/bin/python3 -m pytest test -q` - exit 0, **203 passed** inside the
     packaged copy.

Full logs are not stored separately; the commands above are re-runnable in the
workspace and the packaged copy.

## Outstanding risks and limits

- No ROS 2 Lyrical, Gazebo Jetty or Ubuntu runtime on this host: nothing here is
  Gazebo-validated. Contact dynamics, the rendered 60 Hz camera, actual
  AprilTag decode rate at the decision stops, the 22 rung crossings, the
  measured 180° turn and the complete physical mission still need the VM run
  (`ros2 run wafer_transport_sim four_room_check --scenario nominal`).
- Marker visibility evidence is a static projection of generated geometry, not a
  rendered frame; lighting, exposure and motion blur are unverified.
- The in-place turns at each rung are asserted clear by geometry (19-23 mm), but
  no physics engine has executed them.
- Wheel-slip-free odometry is assumed, as before; the harness is kinematic.

## Decisions for Astra

- `mission_timeout` 900 → 1800 s: the two-track route takes ~11 rooms × ~75 s
  plus an ~86 s return, so the old budget faulted a healthy mission. Revert if
  the VM should instead shorten per-room handling.
- Junctions use in-place turns at the rung mouth rather than arcs, because a
  0.3048 m lane pitch cannot fit the 0.164 m minimum turn radius at the
  configured speed.
- `/transport/track` reports the track a rung *selects* (TRACK_2 for
  `rung_in_*`, TRACK_1 for `rung_out_*`).

No further checkpoint: this bundle is complete and ready for the batched review.
