# Worker checkpoint: two-track FSM

Status: **complete** (implementation worker). See `WORKER_REPORT.md` in this
directory for the final report; this file is the interrupted-run checkpoint and
is no longer live.

## Contract read from PLAN.md

- Replace the repeated raised U detours with two continuous straight black
  lanes (Track 1 main, farther from doors; Track 2 service, beside doors) joined
  by black transverse junctions outside door/box envelopes. No red tape, no
  third return lane.
- FSM owns the active route edge and the junction decision. `/cmd_vel` owner
  stays `transport_controller`. IR is primary on straights, explicit bounded
  junction steering follows the selected edge, pose fallback may steer the
  selected edge but may never fabricate tag authorisation.
- Entry tag + entry FSM authorise Track1 -> Track2; exit tag + completed service
  FSM authorise Track2 -> Track1; the return-home FSM stays on Track 1 and
  ignores service tags. No nearest-projection jumps across parallel lanes.
- Tags are freshness/state scoped per room and decision, never a global set.
  Markers must be visible to the real forward camera before the decision point.
- Diagnostics: transient-local `/transport/track` (TRACK_1|TRACK_2) and
  `/transport/route_segment`, explicit junction/wait FSM states.

## Decisions

- Network: Track 1 at `MAIN_Y` from `START[0]` to `TURN_X`; Track 2 at the
  existing `route_y` between the first and last rung; 22 transverse rungs at
  `room.x +- .45` (outside the 0.3724 m door opening and below the box front
  plane).
- Per room: approach on Track 1 -> decision stop -> confirmed entry tag ->
  rung up -> service lane -> entry door -> transfer -> exit door -> transfer ->
  rung down -> Track 1. After room 11: measured 180 deg turn at the end of
  Track 1, return home on Track 1.
- Units, generator, IR sampling, controller, checker, tests and README updated
  together; release marker changes; ZIP regenerated.

## Result

1. Layout network + 46-edge route plan - done
   (`four_room_layout.py`, `EDGES`/`route_plan`, `MISSION_LENGTH` 41.024 m).
2. Controller FSM/tag scoping/selected-edge navigation - done
   (`four_room_controller.py`, transient-local `/transport/track` +
   `/transport/route_segment`).
3. World generator tape + marker placement - done
   (`generate_four_rooms.py` draws 24 tape visuals and 22 markers).
4. Checker, tests, README, release marker, ZIP - done
   (marker `eleven-glovebox-two-track-fsm-20260923`, 203 tests pass locally and
   from the packaged archive).
5. Overhead preview + report - done
   (`two-track-overhead.png` 1517x199, `WORKER_REPORT.md`).

Remaining limitation: no ROS 2 / Gazebo runtime locally, so physical VM
validation of contact dynamics, rendered tags and the 60 Hz camera is still
outstanding.
