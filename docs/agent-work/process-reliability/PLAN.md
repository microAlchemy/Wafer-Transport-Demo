# Continuation: internal process robot reliability
Root: GPT-6 Astra (active session metadata). Worker: astra_flash_builder, DeepSeek API V4.1 Flash high; static-ready, runtime initially unverified.
Preserve the existing eleven 4ft x 3ft x 4ft gloveboxes, 1ft corridors, exterior IR Raspbot route and AprilTag interfaces.
Baseline: /tmp/wafer-process-baseline-20260923 includes current dirty source/assets, not just HEAD.

## Phase 1 contract
Repair internal entry -> blue stage -> exit transfers without teleportation. Derive cup contact and lift geometry from wafer/support heights. All process detachable joints start detached, and attachment ownership must not conflict with the mobile vacuum or another cell. Existing topic names remain compatible.
Active process execution must fault on stale required feedback, motion/attachment timeout or lost wafer/support; no false ready. Processing dwell requires verified supported placement. Fault stops commanded motion and propagates to transport using existing fault mechanism. Preserve safe attachment retention on faults.
Worker owns generator, generated process assets, process controller, narrowly required shared/controller/config integration, tests, README accuracy and release archive. Do not redesign routing or add new features. Do not commit, push, change model/router config or claim ROS/Gazebo execution on macOS.
Acceptance: geometry/contact regression tests from generated SDF; startup attachment exclusivity; active missing-feedback and attachment failure tests; blue-stage verification; existing suite remains passing. Record commands/results and unexecuted Ubuntu physical validation. Regenerate distributable with distinct release marker if changes pass.

## Remaining integration boundary
Actual Gazebo/Ubuntu validation remains required. Later route compliance review must verify state-dependent AprilTag junction decisions and alignment with the aerial reference; do not represent the whole project as runtime-validated.

## Review
Astra reviews worker delta against baseline under specification and quality lenses, with targeted checks only. One consolidated correction if needed.
