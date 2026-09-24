# Yahboom Raspbot V2 simulation model

The ten-glovebox U-layout demonstration uses a locally generated reconstruction of the
Yahboom Raspbot V2 Standard Kit with Raspberry Pi 5. The stock robot envelope,
wheelbase, weight, camera field of view, pan/tilt range, and motor speed come
from Yahboom's product page and dimension drawing:

- [Raspbot V2 product page](https://category.yahboom.net/products/raspbot-v2)
- [Yahboom dimension drawing](https://cdn.shopify.com/s/files/1/0066/9686/1780/files/Yahboom_Raspbot_V2_AI_Robot_Car_Kit_Details_29.jpg)
- [Yahboom Raspbot V2 study material](https://www.yahboom.net/study/RASPBOT-V2)

| Property | Model value | Source |
|---|---:|---|
| Stock envelope | 208.74 × 165.83 × 127.05 mm | Yahboom drawing |
| Axle wheelbase | 117.20 mm | Yahboom drawing |
| Standard kit mass | 885 g | Yahboom drawing |
| Horizontal camera FOV | 110° | Yahboom drawing |
| Pan / tilt range | 180° / 110° | Yahboom drawing |
| Motor no-load speed | 245 rpm, ±10% | Yahboom drawing |

Yahboom's public STEP download was unavailable because its hosting quota was
exceeded. The wheel radius and width, collision shapes, component placement,
and mass distribution are therefore engineering estimates based on the
published dimensions and product photographs. The generated model is a
dimensioned visual and dynamic reconstruction, not the manufacturer's exact
CAD. These assumptions are isolated in `wafer_transport_sim/raspbot_spec.py`.

Gazebo uses four independently driven wheel joints and the Harmonic/Jetty
`MecanumDrive` system. The black diagonal rollers are represented visually;
anisotropic wheel contact supplies their idealized rolling direction. The
camera pan and tilt are actuated joints. The four visible front IR probes form
the primary line-following array. Because Gazebo has no stock colour-reflectance
IR sensor, `ir_line_follower` combines their physical offsets, the live robot
pose, and the generated tape geometry to publish `/ir/values`, `/ir/valid`, and
`/ir/cmd_vel`. A hardware driver can replace that node while keeping the same
topic contract. The ultrasonic modules, OLED, and RGB lamps are visual geometry
only and do not publish sensor data.

If probe readings stop or cannot provide a forward command, the glovebox
controller can emulate line following from live model pose and the same tape
geometry. This is the explicit simulation recovery requested for the demo,
reported as `SIMULATED_IR` on `/transport/steering_mode`. It is not a physical IR
measurement or a hardware navigation solution. The forward camera gates each
branch and door stop through confirmed AprilTag observations.
The forward pan/tilt camera runs at 640 × 480 and 60 Hz in this simulation so
`apriltag_detector` can read the 22 AprilTag 36h11 entry/exit markers. Actual
range and frame rate on the physical Raspberry Pi depend on lighting, exposure,
tag size, lens calibration, and available compute; “any distance” detection is
not physically possible.

Each glovebox also contains a separate idealized X/Z rail robot. Those internal
robots are facility equipment, not part of the stock Raspbot. They move the
wafer from the entry handoff to the blue process stage and then to the exit
handoff using their own detachable-joint attachment.

The telescoping vacuum wand, carrier tray, detachable joints, and downward line
camera are the wafer-demo payload. They are not stock Yahboom parts. The real
robot will need a mechanically designed payload, calibrated mass and inertia,
hardware drivers, wheel calibration, and safety validation before this
simulation controller can be used on hardware.
