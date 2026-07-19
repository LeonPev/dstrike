# Two-Drone Mountain Scenario Plan

Goal: add a deterministic two-drone scenario on top of the existing Dockerized ArduPilot + MuJoCo + ROS 2 + RViz2/noVNC stack, using the completed mountain terrain map.

Plan-specific outcome: `CAMERA_ENABLED=1 make test-e2e-two-drone`, followed by `CAMERA_ENABLED=1 make up-two-drone-viz`, should prove and display one scripted target drone flying a straight terrain-parallel path while one ArduPilot-controlled chaser drone takes off, intercepts it, and holds formation beside it.

## Status and scope

This is a standalone `SCN-*` plan. It starts only after the base Docker/RViz stack and the mountain-map MVP are already complete.

Required prerequisites:

```text
PASS DFD-000
PASS DFD-014
PASS DFD-016
PASS DFD-018
PASS: MAP-006
```

In scope:

```text
one scripted MuJoCo target drone already in the sky
one ArduPilot-controlled chaser drone starting on the ground
target straight XY flight with constant terrain clearance over the mountain map
chaser takeoff, intercept, and side-by-side formation hold
per-drone odometry, TF, markers, and front camera topics
RViz2/noVNC local-only visualization
bounded Docker-only tests and cleanup
```

Out of scope for the MVP:

```text
second ArduPilot SITL instance
real hardware
cloud dashboards
external map downloads
multi-agent collision avoidance beyond simple separation/geofence checks
```

`SCN-010` is an optional later dual-ArduPilot SITL extension. Do not start it until `SCN-000` through `SCN-009` pass.

## Repository gate before SCN-000

Do not start implementation until the checkout has no unrelated changes and the completed base plus map stack is present.

Required gate:

```bash
git status --short
# expected: no unrelated modified/deleted files
# allowed before SCN-000: the canonical untracked plans/two_drone_mountain_scenario_plan.md draft itself
# allowed in SCN-000 only: AGENTS.md reroute, README link, Makefile placeholders, verifier script
```

Retain these existing plan files if they are tracked in the repo:

```text
plans/ardupilot_mujoco_ros2_full_docker_plan.md
plans/mountain_terrain_map_plan.md
plans/two_drone_mountain_scenario_plan.md
```

Branching rule:

```text
Do not assume `main` is the correct base.
Start SCN-000 from the latest committed tip of the completed MAP-006 stack.
Each later SCN branch starts from the committed or merged predecessor SCN branch.
```

Example only, to be run from the current committed MAP-006 tip, not from stale `main`:

```bash
git log --oneline -1
# confirm this is the latest completed MAP-006 tip

git checkout -b feature/SCN-000-two-drone-plan
```

Host `git` and `make` are allowed. ROS 2, MuJoCo, ArduPilot, and robotics Python commands must run through Docker.

## Existing repo contract

- Keep existing base tickets as `DFD-*` and map tickets as `MAP-*`. This plan uses only `SCN-*`.
- Docker Desktop on macOS is the only host prerequisite.
- Use `/workspace` inside containers.
- Keep `scenarios.py` as the single scenario entry point.
- Current scenario XML resolution is `models/scenarios/<scenario>.xml`; extend this, do not bypass it.
- Reuse the completed `mountain_map` terrain artifacts and terrain APIs from `MAP-*`; do not regenerate or redefine the map in this plan.
- Do not add `/clock` in this scenario milestone.
- Preserve `/sim/scenario_markers`.
- Preserve `/sim/drone_markers`.
- Preserve existing single-drone topics for non-two-drone scenarios, especially `/drone/odom` and `/sim/ground_truth/odom`.
- For `SCENARIO=mountain_two_drones`, `/drone/odom` remains the ArduPilot-controlled chaser alias.
- Keep ArduPilot JSON boundary behavior from the base plan: NED at the JSON boundary, z-up in MuJoCo/RViz, quaternion-only attitude.
- Camera env var remains `CAMERA_ENABLED`; do not introduce `ENABLE_CAMERAS`.
- Every new Make target must be listed in `.PHONY` and `make help`.
- Placeholder help text must be replaced when each target becomes real.
- Every automated verifier prints exactly one success line only after all assertions pass:

```text
PASS: SCN-XXX ...
```

## Implementation notes for Codex

- Codex owns narrow implementation details within each ticket. It may choose the simplest repo-native design for helper functions, internal class/API shape, motion interpolation, flight-control implementation, DDS/reset correlation, probe geometry, dependency organization, and cleanup mechanics, provided every stated contract, backward-compatibility requirement, bounded verifier, regression, Definition of Done, and exact PASS line succeeds.
- Codex must update the plan before proceeding only when a decision would change a public interface, Make target, ROS topic, environment variable, artifact format, coordinate convention, ticket scope/order, or acceptance behavior.
- Do not add low-level implementation prescriptions when a bounded observable verifier is enough.
- Do not remove observable contracts, regression requirements, or exact PASS lines.

## Researched design constraints

Primary references:

```text
ArduPilot JSON external-physics interface:
https://ardupilot.org/dev/docs/sitl-with-JSON.html

ArduPilot multi-vehicle/SYSID guidance for optional later dual-SITL work:
https://ardupilot.org/mavproxy/docs/getting_started/multi.html

MuJoCo body camera XML reference:
https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-camera

ROS 2 RViz marker display types:
https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html
```

Design decisions:

- Use one ArduPilot-controlled chaser plus one scripted MuJoCo target for the MVP. This avoids two SITL instances, duplicate MAVLink routing, duplicate JSON physics channels, and `SYSID_THISMAV` coordination until the visible behavior is proven.
- Keep the scripted target kinematic and deterministic. It should not be affected by ArduPilot or the chaser.
- Use ArduPilot JSON external physics only for the chaser, preserving the existing base SITL path.
- Upgrade chaser physics to per-rotor force/torque if the current model still applies summed thrust at the body center; lateral chase behavior requires roll/pitch/yaw authority.
- For this plan, “straight line parallel to the ground” means straight in XY while maintaining constant terrain clearance above the mountain map: `z_world = terrain_height(x, y) + altitude_agl_m`.
- Use MuJoCo cameras attached to each drone body for perspective views, published as ROS image topics and displayed in RViz2 through noVNC.
- RViz2 remains the only visualization UI; no external cloud dashboard is required.

## Coordinate, frame, and public-topic contract

World convention:

```text
MuJoCo/RViz world frame: z-up
ArduPilot JSON boundary: NED, converted at the existing boundary
heading_deg: 0° = +X, positive counter-clockwise around +Z, normalized modulo 360
```

Required public topics for `SCENARIO=mountain_two_drones`:

```text
/drone/odom                                  nav_msgs/Odometry, chaser alias/backward compatibility
/sim/ground_truth/odom                       nav_msgs/Odometry, chaser alias/backward compatibility
/sim/chaser/ground_truth/odom                nav_msgs/Odometry
/sim/target/odom                             nav_msgs/Odometry
/sim/scenario_markers                        visualization_msgs/MarkerArray
/sim/drone_markers                           visualization_msgs/MarkerArray
/tf                                          tf2_msgs/TFMessage
/scenario/start                              std_msgs/Bool
/scenario/state                              std_msgs/String
/scenario/formation_error_m                  std_msgs/Float32
```

Camera topics when `CAMERA_ENABLED=1`:

```text
/sim/target/camera/front/image_raw           sensor_msgs/Image
/sim/target/camera/front/camera_info         sensor_msgs/CameraInfo
/sim/chaser/camera/front/image_raw           sensor_msgs/Image
/sim/chaser/camera/front/camera_info         sensor_msgs/CameraInfo
```

Required TF frames:

```text
world
  target/base_link
  target/front_camera
  chaser/base_link
  chaser/front_camera
```

If the repo has an existing frame alias that must be preserved for backward compatibility, keep it in addition to these frames rather than replacing these frames.

## Make target ownership

```text
up-two-drone-viz       placeholder in SCN-000, real implementation in SCN-007
test-e2e-two-drone     placeholder in SCN-000, real implementation in SCN-009
```

`generate-map`, `test-map`, and `up-map-viz` remain owned by the `MAP-*` plan. Do not reassign them here.

---

## SCN-000 — Add this two-drone plan and safe placeholders

**Codex task**

Add this plan to the repository and create safe placeholder targets.

Files:

```text
plans/two_drone_mountain_scenario_plan.md
AGENTS.md
README.md
Makefile
scripts/verify_scn000_placeholders.py
```

Requirements:

- Add a README section linking to this plan.
- If `AGENTS.md` routes Codex to the active plan, update it to point to this plan only for the two-drone work.
- Add placeholder Make targets that fail clearly until implemented:
  - `up-two-drone-viz`
  - `test-e2e-two-drone`
- Placeholder output must say exactly which later SCN ticket implements the real target.
- Placeholder failures must be intentional and testable, not `command not found` or missing-target failures.
- Add both targets to `.PHONY` and `make help` with placeholder wording.
- Do not claim the two-drone scenario is runnable yet.

**Definition of Done**

```bash
test -f plans/two_drone_mountain_scenario_plan.md
docker compose config
make help
python3 scripts/verify_scn000_placeholders.py
```

The verifier must prove:

```text
up-two-drone-viz exists, fails intentionally, and mentions SCN-007
test-e2e-two-drone exists, fails intentionally, and mentions SCN-009
README links this plan
no SCN ticket has been implemented yet
```

Done when the verifier exits `0` and prints exactly:

```text
PASS: SCN-000 two-drone plan and placeholder targets are present
```

---

## SCN-001 — Add and validate the two-drone scenario config

**Codex task**

Create a validated scenario config that reuses the completed mountain map.

Files:

```text
config/scenarios/mountain_two_drones.yaml
src/physics/scenario_config.py
scripts/verify_two_drone_config.py
tests/test_two_drone_config.py
```

Minimum config shape:

```yaml
name: mountain_two_drones
frame_id: world
map:
  name: mountain_map

target_drone:
  name: target
  start_xy_m: [-80.0, 0.0]
  heading_deg: 0.0
  speed_mps: 6.0
  altitude_agl_m: 35.0
  route_length_m: 120.0

chaser_drone:
  name: chaser
  start_xy_m: [-120.0, -25.0]
  heading_deg: 0.0
  initial_agl_m: 0.3
  takeoff_agl_m: 12.0
  formation_offset_target_body_m: [-8.0, -5.0, 0.0]

safety:
  mission_timeout_s: 120.0
  min_terrain_clearance_m: 5.0
  min_drone_separation_m: 3.0
  max_chaser_speed_mps: 12.0
  geofence_margin_m: 5.0

visualization:
  camera_rate_hz: 5.0
  marker_rate_hz: 5.0
```

Validation requirements:

- Reject unknown keys.
- Reject non-finite numeric values.
- Require `name == mountain_two_drones`.
- Require `frame_id == world`.
- Require `map.name == mountain_map`.
- Require positive target speed, route length, mission timeout, camera rate, and marker rate.
- Require nonnegative AGL values where applicable.
- Require chaser takeoff AGL greater than initial AGL.
- Normalize heading modulo `360` using the shared convention: `0° = +X`, positive counter-clockwise around `+Z`.
- Test heading conversion for `0°`, `90°`, `-90°`, `180°`, and `450°`.
- Require both spawn points and the full target route to remain inside the MAP geofence margin.
- Require the target route to maintain at least `safety.min_terrain_clearance_m` over the terrain.
- Require the configured formation offset norm to be at least `safety.min_drone_separation_m`.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_config.py
docker compose run --rm physics python3 scripts/verify_two_drone_config.py --scenario mountain_two_drones
```

The verifier must print resolved target/chaser spawn poses, target route endpoints, and the normalized headings.

Done when both commands exit `0` and the verifier prints exactly:

```text
PASS: SCN-001 mountain_two_drones config validates against the completed mountain map
```

---

## SCN-002 — Add two-drone MuJoCo world composition

**Codex task**

Extend the existing scenario loader so `SCENARIO=mountain_two_drones` loads the mountain map plus two drone entities.

Files:

```text
models/scenarios/mountain_two_drones.xml
models/common/quad_visual.xml
src/physics/scenarios.py
src/physics/two_drone_world.py
scripts/verify_two_drone_world.py
tests/test_two_drone_world.py
tests/test_scenarios.py
```

Requirements:

- Keep `scenarios.py` as the canonical scenario loader.
- Keep the entry path `models/scenarios/mountain_two_drones.xml`.
- Reuse the MAP terrain asset loading path and `mountain_map` terrain artifacts.
- Add a chaser drone body controlled by the existing ArduPilot JSON physics path.
- Add a target drone body controlled only by the scripted target-motion path.
- Reuse shared visual quadrotor geometry where practical.
- Ensure body, geom, site, actuator, camera, and sensor names are unique.
- For the MVP, configure the target drone as non-contacting or collision-filtered so it cannot destabilize the chaser; terrain contact for the chaser must still work.
- Preserve the existing default/landing-pad scenarios and map scenario behavior.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_scenarios.py tests/test_two_drone_world.py
docker compose run --rm physics python3 scripts/verify_two_drone_world.py --scenario mountain_two_drones
```

The verifier must assert:

```text
one target body exists
one chaser body exists
both spawn above the mountain terrain using MAP height queries
all public body/camera/site names are unique
target collision policy is explicit
chaser-terrain contact still works
landing_pad and mountain_map scenarios still load
```

Done when both commands exit `0` and the verifier prints exactly:

```text
PASS: SCN-002 two-drone MuJoCo world composes with the existing mountain terrain
```

---

## SCN-003 — Upgrade chaser dynamics for lateral flight

**Codex task**

Ensure the ArduPilot-controlled chaser has enough physics authority to move laterally, yaw, and track the target. If the current implementation already passes the tests, do not rewrite it.

Files:

```text
src/physics/mujoco_quad.py
src/physics/motor_model.py
tests/test_motor_mixing.py
tests/test_quad_attitude_response.py
tests/test_mujoco_quad.py
scripts/verify_chaser_dynamics.py
```

Requirements:

- Apply rotor forces at rotor-specific sites, not only as summed center-body thrust, unless an existing higher-fidelity implementation already provides equivalent tested behavior.
- Define documented motor order and yaw torque signs.
- Preserve existing PWM input behavior from ArduPilot JSON.
- Preserve the DFD takeoff smoke test behavior.
- Add quantitative tests for:
  - symmetric motor command produces positive vertical acceleration and low angular acceleration;
  - roll imbalance produces the expected roll angular acceleration sign;
  - pitch imbalance produces the expected pitch angular acceleration sign;
  - yaw imbalance produces the expected yaw angular acceleration sign;
  - non-commanded axes remain below `10%` of the commanded-axis response in each axis-specific test;
  - commanded-axis angular acceleration magnitude is greater than `0.05 rad/s^2` in the axis-specific tests.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_motor_mixing.py tests/test_quad_attitude_response.py tests/test_mujoco_quad.py
docker compose run --rm physics python3 scripts/verify_chaser_dynamics.py
SCENARIO=landing_pad make test-e2e
```

Done when all commands exit `0` and the verifier prints exactly:

```text
PASS: SCN-003 chaser dynamics have tested vertical, roll, pitch, and yaw authority
```

---

## SCN-004 — Script the sky drone's straight terrain-parallel flight

**Codex task**

Implement the target drone motion script.

Files:

```text
src/physics/target_motion.py
scripts/target_motion_probe.py
tests/test_target_motion.py
```

Behavior contract:

```text
target starts at the configured sky pose
target moves in a straight XY line along configured heading
target speed equals configured speed within tolerance
target keeps z = terrain_height(x, y) + altitude_agl_m
target yaw follows the direction of travel
target publishes odometry and TF continuously during live simulation
```

Test thresholds:

```text
cross_track_error_m <= 0.25
speed_error_mps <= 0.25
AGL_error_m <= 0.50
heading_error_deg <= 2.0
minimum_terrain_clearance_m >= configured altitude_agl_m - 0.50
```

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_target_motion.py
docker compose run --rm physics python3 scripts/target_motion_probe.py --scenario mountain_two_drones --duration-s 15
```

Done when both commands exit `0` and the probe prints exactly:

```text
PASS: SCN-004 target drone flies a straight terrain-parallel route
```

---

## SCN-005 — Publish two-drone odometry, TF, and markers

**Codex task**

Publish namespaced live telemetry and visualization markers for both drones while preserving existing topics.

Files:

```text
src/physics/ros_visualization.py
src/physics/ros_telemetry.py
scripts/two_drone_topic_probe.py
tests/test_ros_visualization.py
tests/test_two_drone_topics.py
```

Required topics:

```text
/sim/target/odom
/sim/chaser/ground_truth/odom
/sim/ground_truth/odom
/drone/odom
/sim/scenario_markers
/sim/drone_markers
/tf
```

Marker requirements:

- `/sim/scenario_markers` remains present and includes terrain/map scenario markers from MAP.
- `/sim/drone_markers` includes distinguishable target, chaser, target route, and desired formation-point markers.
- Static markers use transient-local delivery where appropriate.
- Odom topics use live volatile delivery.
- All marker points and odometry poses are finite.
- Marker frame IDs are `world`.
- Chaser alias topics match the chaser pose within `0.05 m` position tolerance and `1°` yaw tolerance.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_ros_visualization.py tests/test_two_drone_topics.py
SCENARIO=mountain_two_drones docker compose --profile mujoco up -d sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/two_drone_topic_probe.py --timeout 60'
make down
```

The probe must check late-subscriber transient-local delivery for markers and at least `5` live odometry messages for each drone.

Done when all commands exit `0` and the probe prints exactly:

```text
PASS: SCN-005 target and chaser telemetry, TF, aliases, and markers publish correctly
```

---

## SCN-006 — Add front camera perspectives for both drones

**Codex task**

Attach one front-facing MuJoCo camera to each drone and publish both image streams when `CAMERA_ENABLED=1`.

Files:

```text
models/scenarios/mountain_two_drones.xml
src/physics/camera_publisher.py
scripts/two_drone_camera_probe.py
tests/test_two_drone_cameras.py
```

Required camera names:

```text
target_front_cam
chaser_front_cam
```

Required ROS topics:

```text
/sim/target/camera/front/image_raw
/sim/target/camera/front/camera_info
/sim/chaser/camera/front/image_raw
/sim/chaser/camera/front/camera_info
```

Requirements:

- Use `CAMERA_ENABLED=1` to enable image publication.
- Keep the default rate at `visualization.camera_rate_hz` from config.
- Keep image resolution low enough for Docker/noVNC performance.
- Publish valid `CameraInfo` for each camera.
- The target and chaser image streams must be nonblank and not byte-identical across at least `3` paired frames.
- Preserve the existing single-drone camera topic in non-two-drone scenarios.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_cameras.py
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones docker compose --profile mujoco up -d sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/two_drone_camera_probe.py --timeout 60 --min-frames 3 --nonblank --require-different'
make down
```

Done when all commands exit `0` and the probe prints exactly:

```text
PASS: SCN-006 target and chaser front camera perspectives publish valid nonblank images
```

---

## SCN-007 — Add RViz2/noVNC two-drone visualization target

**Codex task**

Replace the placeholder `up-two-drone-viz` with a real Docker-only visualization target.

Files:

```text
config/two_drone_mountain.rviz
compose.yaml
Makefile
scripts/start_rviz_novnc.sh
scripts/two_drone_viz_probe.py
README.md
```

Required RViz displays:

```text
Fixed Frame: world
TF display
Odometry: /sim/target/odom
Odometry: /sim/chaser/ground_truth/odom
Odometry: /drone/odom
MarkerArray: /sim/scenario_markers
MarkerArray: /sim/drone_markers
Image: /sim/target/camera/front/image_raw
Image: /sim/chaser/camera/front/image_raw
```

Target behavior:

- `make up-two-drone-viz` starts the stack detached with `SCENARIO=mountain_two_drones`.
- It uses `config/two_drone_mountain.rviz`.
- It respects `CAMERA_ENABLED`; examples should use `CAMERA_ENABLED=1`.
- noVNC remains bound to localhost only.
- The wrapper performs deterministic pre-cleanup, bounded startup checks, and fails clearly if required topics do not appear.
- The target remains running for manual viewing until `make down`.

**Definition of Done**

```bash
CAMERA_ENABLED=1 make up-two-drone-viz
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/two_drone_viz_probe.py --timeout 60'
make down
```

Manual check:

```text
Open http://localhost:6080/vnc.html?autoconnect=true&resize=scale
Confirm RViz2 shows mountain terrain, target drone, chaser drone, route/formation markers, and two image panels.
```

Done when the automated probe exits `0`, the manual check is successful, and the probe prints exactly:

```text
PASS: SCN-007 two-drone RViz2/noVNC view works locally
```

---

## SCN-008 — Add chaser mission state machine

**Codex task**

Create a ROS 2 scenario controller that commands the ArduPilot-controlled chaser from ground start to intercept mode.

Files:

```text
src/ros2/drone_bridge/scenario_controller.py
scripts/scenario_state_probe.py
tests/test_scenario_controller.py
```

Subscribed topics:

```text
/scenario/start
/sim/target/odom
/drone/odom
```

Published topics:

```text
/scenario/state
/scenario/formation_error_m
```

Use the existing command topics from the base plan:

```text
/drone/arm
/drone/takeoff_alt_m
/drone/cmd_vel
```

State machine:

```text
IDLE -> HEALTH_CHECK -> ARM -> TAKEOFF -> INTERCEPT -> FORMATION_HOLD -> COMPLETE
IDLE -> ... -> ABORT on timeout, health failure, geofence violation, or unsafe clearance
```

Requirements:

- Use the DFD pre-arm/EKF health gate before arming.
- Use finite timeouts for every state transition.
- Use altitude gain over the terrain/spawn pose, not absolute `z > threshold`, to confirm takeoff.
- Enforce geofence, minimum terrain clearance, maximum commanded speed, and minimum target separation.
- Publish `/scenario/state` changes immediately and keep the latest state observable.
- Do not command direct motors from ROS; use the existing ArduPilot-guided command path.

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_scenario_controller.py
CAMERA_ENABLED=1 make up-two-drone-viz
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic pub --once /scenario/start std_msgs/msg/Bool "{data: true}"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/scenario_state_probe.py --timeout 75 --expect INTERCEPT --min-altitude-gain-m 5.0'
make down
```

Done when all commands exit `0` and the probe prints exactly:

```text
PASS: SCN-008 chaser mission state machine reaches INTERCEPT safely
```

---

## SCN-009 — Implement intercept, formation hold, and e2e test

**Codex task**

Implement the guidance behavior and replace the placeholder `test-e2e-two-drone` with a real bounded end-to-end test.

Files:

```text
src/ros2/drone_bridge/formation_guidance.py
scripts/formation_probe.py
scripts/e2e_mountain_two_drone.py
Makefile
README.md
tests/test_formation_guidance.py
```

Behavior contract:

```text
during INTERCEPT, command the chaser toward the target's desired formation point
during FORMATION_HOLD, keep the chaser beside the target using formation_offset_target_body_m
target continues straight terrain-parallel flight during the maneuver
chaser remains inside geofence and above minimum terrain clearance
chaser never intentionally violates minimum drone separation
```

Required metrics:

```text
formation_error_m          distance from chaser to desired formation point
actual_separation_m        distance from chaser to target
terrain_clearance_m        chaser_z - terrain_height(chaser_x, chaser_y)
target_cross_track_error_m target deviation from configured route
chaser_altitude_gain_m     chaser_z - initial_spawn_z
```

End-to-end acceptance thresholds:

```text
target_cross_track_error_m <= 0.50
target_AGL_error_m <= 0.75
chaser_altitude_gain_m >= 5.0
formation_error_m <= 5.0 for at least 10.0 continuous seconds
actual_separation_m >= safety.min_drone_separation_m
terrain_clearance_m >= safety.min_terrain_clearance_m
mission finishes within safety.mission_timeout_s
```

`test-e2e-two-drone` behavior:

- Runs `make down` before starting.
- Starts with `SCENARIO=mountain_two_drones` and honors `CAMERA_ENABLED`.
- Waits with bounded timeouts for telemetry, markers, and optional cameras.
- Publishes `/scenario/start`.
- Verifies target route, chaser takeoff, decreasing formation error, and sustained formation hold.
- Cleans up on success, failure, and interruption.
- Ends with the stack down.

Mandatory regressions in this ticket:

```bash
SCENARIO=landing_pad make test-e2e
make test-map
```

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_formation_guidance.py
CAMERA_ENABLED=1 make test-e2e-two-drone
SCENARIO=landing_pad make test-e2e
make test-map
make down
```

Done when all commands exit `0`, the stack is down afterward, and the e2e test prints exactly:

```text
PASS: SCN-009 target straight terrain-parallel flight, chaser takeoff, intercept, and formation hold verified
```

---

## SCN-010 — Optional later: replace scripted target with second ArduPilot SITL

**Codex task**

Replace the scripted target with a second ArduPilot SITL only after the MVP passes.

Files:

```text
compose.yaml
Makefile
src/physics/multi_vehicle_json.py
src/ros2/drone_bridge/multi_vehicle_bridge.py
scripts/dual_sitl_probe.py
tests/test_dual_sitl_routing.py
```

Requirements:

- Add separate `sitl-target` and `sitl-chaser` services.
- Use unique MAVLink UDP ports.
- Use unique `SYSID_THISMAV` values.
- Route separate JSON physics channels to separate MuJoCo bodies.
- Use separate ROS namespaces for target and chaser command/telemetry paths.
- Preserve the scripted-target MVP path as the default until this optional profile is explicitly selected.

Suggested profile:

```bash
docker compose --profile dual-sitl up -d sitl-target sitl-chaser physics ros2 viz
```

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_dual_sitl_routing.py
docker compose --profile dual-sitl up -d sitl-target sitl-chaser physics ros2 viz
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/dual_sitl_probe.py --timeout 90'
make down
```

Done when both vehicles have separate heartbeats, separate vehicle IDs, separate odometry streams, and the probe prints exactly:

```text
PASS: SCN-010 optional dual-ArduPilot SITL setup works with separate vehicle identities
```

---

## Final command sequence after MAP-006

Run tickets in order:

```text
SCN-000
SCN-001
SCN-002
SCN-003
SCN-004
SCN-005
SCN-006
SCN-007
SCN-008
SCN-009
```

Regression and demo workflow after implementation:

```bash
SCENARIO=landing_pad make test-e2e
make test-map
CAMERA_ENABLED=1 make test-e2e-two-drone
CAMERA_ENABLED=1 make up-two-drone-viz
# open http://localhost:6080/vnc.html?autoconnect=true&resize=scale
make down
```

Milestone readiness:

```text
SCN-002 proves both drones exist in the MuJoCo mountain world.
SCN-004 proves the target sky drone flies the required straight terrain-parallel route.
SCN-006 proves both drone camera perspectives publish.
SCN-007 proves the full scenario is visible in RViz2/noVNC.
SCN-009 proves the complete takeoff, intercept, and formation behavior.
```
