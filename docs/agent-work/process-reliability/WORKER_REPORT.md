# Worker report: internal process robot reliability (Phase 1)

STATUS: ready_for_review

Task: Phase 1 contract in `docs/agent-work/process-reliability/PLAN.md`
Workspace: `/Users/srigan/Wafer-Transport-Demo`
Baseline compared against: `/tmp/wafer-process-baseline-20260923`
Full command output: `docs/agent-work/process-reliability/WORKER_TEST_LOG.txt`

## Findings that drove the change

1. Cup height mismatch. The generated cell parked the vacuum-cup bottom at
   `z = .158` while a wafer on any support sits with its top face at `.156`, and
   the controller then commanded `process_z = -.105`. The cup was therefore
   driven about 103 mm through the wafer and the support platform, and the 0.18 m
   vertical arm overlapped the fixed rail at `.37`. Travel was never derived
   from the wafer or support heights.
2. Startup attachment conflict. Gazebo's `DetachableJoint` system requests
   attachment on its first update (`attachRequested{true}` in `DetachableJoint.hh`),
   so all eleven cells, the mobile vacuum and the carrier welded to the single
   `wafer` model at load, at mutually incompatible offsets. The old controller
   released cells in `IDLE` without verifying acknowledgement, and the transport
   was free to start while those welds still existed.
3. Missing active faults. `tick()` returned silently when required feedback was
   stale, motion and attachment commands had no timeout, a lost wafer/support was
   never checked, the branch to `PROCESSING` did not verify a supported stage
   placement, and `READY_EXIT` could stay true after the wafer left the handoff.

## Changed paths and behavior

Geometry contract (`wafer_transport_sim/wafer_transport_sim/four_room_layout.py:66-91`)
adds the derived chain `SUPPORT_TOP = .154`, `WAFER_THICKNESS = .002`,
`SUPPORTED_WAFER_Z = .155`, `PROCESS_LIFT_STROKE = .03`, from which the rail
height, gripper height, cup rest bottom, contact travel, transit travel and the
arm/rail clearance are computed. `Room.table` and `Room.handoff` now use
`SUPPORTED_WAFER_Z` instead of a repeated literal.

Generator (`wafer_transport_sim/scripts/generate_four_rooms.py:79-110` and the
stage/handoff solids) builds every shuttle number from those constants. The
generated cells now park the cup bottom at `.186` (30 mm above the supported
wafer top), run `z` from `-.03` (contact with the wafer top face) to `0`
(transit), and raise the rail to `.475` so the arm top `.433` clears the rail
bottom `.4475` by 14.5 mm. Model inertia follows the real rail size. The stage
and both handoff shelves are built from `SUPPORT_TOP`.

Regenerated assets: `wafer_transport_sim/models/room_{1..11}_process_robot/model.sdf`
(rail pose `.475`, gripper pose `.293`, cup pose `-.101`, `z` limits `-.03/0`).
Rooms 5-11 were already untracked additions in the dirty baseline; they now
carry the same derived geometry as rooms 1-4. `worlds/four_rooms.sdf`,
`config/four_rooms_bridge.yaml`, textures and door models are byte-identical to
the baseline.

Controller (`wafer_transport_sim/wafer_transport_sim/process_cell_controller.py`):
startup `RELEASE` state parks all eleven cups and publishes
`/process/cells_detached` (line 97-118, 273); `claim()` requires an
acknowledgement within `attachment_timeout` (131-148); `ownership_issues()`
refuses an attach while the mobile vacuum or another cell still holds the wafer
(120-129); `supported()`/`carried()` verify the wafer on the entry handoff, on
the blue stage and on the exit handoff (73-85); `VERIFY_STAGE` requires a
verified supported stage placement before the dwell (190-193) and `PROCESSING`
keeps checking it; `active()` faults on stale required feedback and on
`motion_timeout`/`state_timeout` (242-262); `hold()` re-commands measured joint
positions on every fault tick and never releases the attachment (150-157).
`/process/ready` is non-empty only in `READY_EXIT` with the cup detached and the
wafer verified on the exit handoff.

Transport integration (`wafer_transport_sim/wafer_transport_sim/four_room_controller.py:49,122,421`)
watches `/process/cells_detached` and refuses to leave `WAIT_FOR_SIMULATOR`
until every cell reports detached, with the reason surfaced through
`readiness_issues()`.

Config (`wafer_transport_sim/config/four_rooms.yaml`) adds `motion_timeout: 30.0`
and `attachment_timeout: 5.0` for `process_cell_controller`.

Ubuntu check (`wafer_transport_sim/wafer_transport_sim/four_room_check.py:117,167,185,204`)
adds the `failed_process` scenario, which drops the
`/attachments/process_1/attach` bridge, watches `/process/state`, and requires
the process cell to report the fault without a delivery.
`scripts/verify_ubuntu.sh:19` runs it with the other scenarios.

Tests: new `wafer_transport_sim/test/test_process_cells.py` (14 tests: generated
contact/clearance regression, support-height agreement, startup release and
release-timeout fault, stale feedback, motion timeout, unacknowledged attach,
attach refusal for the mobile vacuum and another cell, blue-stage placement,
lost wafer after pickup, dwell support, external fault retention, full cycle
ready gate, release/scenario consistency). `test/test_four_rooms.py` reports
`/process/cells_detached` from its harness, walks the internal transfer with the
derived `z` targets, and adds a startup-gate test.

Docs, marker and packaging: `README.md` documents the internal cell geometry,
startup release gate, fault contract and the `failed_process` check, and greps
the new marker; `config/release.txt` is
`eleven-glovebox-process-reliability-20260923`;
`dist/wafer-transport-ubuntu26.04.zip` was rebuilt from the working tree.

## Verification

| Command | Exit | Result |
|---|---|---|
| `PYTHONPATH=wafer_transport_sim /usr/bin/python3 -m pytest wafer_transport_sim/test -q` | 0 | 162 passed (baseline: 147 passed) |
| `PYTHONPATH=wafer_transport_sim /usr/bin/python3 -m pytest wafer_transport_sim/test/test_process_cells.py -v` | 0 | 14 passed |
| `cd wafer_transport_sim/scripts && /usr/bin/python3 generate_four_rooms.py` run twice | 0 | tree md5 `03599dc3483b215863433f57eb8eb68a` unchanged; regeneration is byte-stable |
| geometry dump over all 11 generated models | 0 | `z_limits=(-0.03, 0.0)`, `gripper_z=0.293`, cup contact `.156` = wafer top, arm/rail clearance 0.0145 |
| `unzip -Z1 dist/wafer-transport-ubuntu26.04.zip \| wc -l` | 0 | 236 entries, marker `eleven-glovebox-process-reliability-20260923` inside |
| `unzip -t dist/wafer-transport-ubuntu26.04.zip` | 0 | no errors; no `agent-work`, `__pycache__`, `.DS_Store` or `pytest_cache` entries |

Python used: `/usr/bin/python3` 3.9.6 with pytest 8.4.2 (the Homebrew 3.14
interpreter on this Mac has no pytest installed).

## Not verified (physical validation pending)

No ROS 2 or Gazebo exists on this macOS workspace, so none of the following ran
here: `gz sdf -k`, `ros2 launch wafer_transport_sim demo.launch.py`,
`four_room_check` (including `--scenario failed_process`), `verify_ubuntu.sh`,
detachable-joint physics, contact dynamics, joint controllers, camera/AprilTag
rendering and IR probes. The nominal mission is still not runtime-validated, and
no claim is made that the whole project runs on Ubuntu.

## Outstanding risks

1. Load-time welds. `DetachableJoint` attaches on its first update, so for the
   first controller ticks the eleven cups are still welded to the wafer. The
   controller releases them at startup and the transport now refuses to start
   until `/process/cells_detached` is true, but if the drag is large the wafer
   may leave the carrier before the mission starts. Check the wafer/carrier pose
   and `/system/fault` right after launch in the VM. There is no supported SDF
   switch to suppress the initial attach in this plugin.
2. Derived stroke. The 30 mm lift stroke and the raised rail are the smallest
   change that lets the existing arm reach the wafer without intersecting the
   rail or the support. The visual silhouette of the internal shuttle changed;
   the constants in `four_room_layout.py:66-91` are the single place to retune if
   the VM shows a residual contact or a preferred shorter arm.
3. Startup gating. If the process controller or its attachment bridge never
   reports, the mission will not start; the transport reports this through
   `readiness_issues()` and its 90 s wall-clock startup timeout. This is the
   intended fail-safe, but it changes startup behaviour for anyone running the
   four_room layout without those bridges.
4. Blue-stage dwell uses a fixed `process_hold`; there is no verified thermal or
   process-time model, matching the previous scope.

## Decisions requiring Astra

1. Accept the derived rail/gripper heights (visual change to the internal
   shuttle) as the Phase 1 fix, or ask for a shorter-arm variant instead.
2. Confirm whether the `failed_process` scenario in `four_room_check` is
   sufficient evidence for the process attachment fault during Ubuntu
   acceptance, or whether a dedicated process-only check is wanted.
3. Confirm the new release marker/archive naming is the intended distributable
   for this phase.

## Next checkpoint

On the Ubuntu VM: `bash wafer_transport_sim/scripts/verify_ubuntu.sh`, then
report `/process/state`, `/process/ready`, `/process/cells_detached` and
`/system/fault` plus the `four_room_check` JSON for `nominal` and
`failed_process`. Until then the physical half of the acceptance list stays
open.
