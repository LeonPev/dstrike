# Two-Drone Mountain Scenario Plan

Repository location: `plans/two_drone_mountain_scenario_plan.md`

Goal: add a deterministic two-drone scenario on top of the Dockerized ArduPilot + MuJoCo + ROS 2 + RViz2/noVNC stack, using the completed mountain terrain map.

Plan-specific outcome: `CAMERA_ENABLED=1 make test-e2e-two-drone`, followed by `CAMERA_ENABLED=1 make up-two-drone-viz`, proves and displays one scripted target drone flying a straight terrain-parallel route while one ArduPilot-controlled chaser drone takes off, intercepts it, and holds formation beside it.

## Status and scope

This is a standalone `SCN-*` plan. Start it only after the base Docker/RViz stack and the mountain-map MVP are complete.

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
active multi-agent collision avoidance
```

`SCN-010` is an optional later dual-ArduPilot SITL extension. Do not start it until `SCN-000` through `SCN-009` pass.

## Repository gate before SCN-000

Do not start implementation until the checkout has no unrelated changes and the completed base plus map stack is present.

Required gate:

```bash
git status --short
# expected: no unrelated modified/deleted files
# allowed before SCN-000: the canonical plans/two_drone_mountain_scenario_plan.md draft itself, if SCN-000 is the bootstrap commit that adds this plan
# allowed in SCN-000 only: plans/two_drone_mountain_scenario_plan.md, AGENTS.md reroute, README link, Makefile placeholders, verifier script
```

If the repository is intentionally migrating active plans into `plans/`, commit that migration coherently before `SCN-000`. Reconcile and commit final plan corrections before creating the SCN-000 implementation branch unless `SCN-000` is explicitly the bootstrap commit that adds this exact canonical plan file. Do not begin SCN implementation from a tree with unrelated modified/deleted plan files, deleted base plans, an uncommitted `plans/` migration, or uncommitted final plan corrections.

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

Host requirements remain limited to Docker Desktop, Docker Compose, `make`, Git, a shell, and a browser. ROS 2, MuJoCo, ArduPilot, and robotics Python commands must run through Docker; do not require host `python3`.

## Existing repo contract

- Keep existing base tickets as `DFD-*` and map tickets as `MAP-*`. This plan uses only `SCN-*`.
- Use `/workspace` inside containers.
- Keep `scenarios.py` as the single scenario entry point.
- Current scenario XML resolution is `models/scenarios/<scenario>.xml`; extend this path, do not bypass it.
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
- Verifiers may print diagnostic information but must print exactly one `PASS:` line, as their final line, only after all assertions succeed:

```text
PASS: SCN-XXX ...
```

Live-stack cleanup contract:

- Any ticket that starts long-running Compose services must clean up on success, failure, and interruption. SCN-005 through SCN-008 live checks must use a wrapper, Make target, or shell trap equivalent to `make down` on `EXIT`, `INT`, and `TERM`.
- SCN-007 has two modes: `scripts/run_scn007_viz_check.sh` is the automated verifier and must leave the stack down; `make up-two-drone-viz` is the separate manual-viewing launch and may leave the stack running only after bounded startup/probe checks succeed.
- Failed startup/probe checks must run `make down` before exiting nonzero.
- Definition-of-Done command blocks are acceptance sequences, not permission to leave containers running after a failed intermediate command.
- Cleanup output must not obscure the verifier contract: each automated verifier's exact `PASS:` line must remain the final line only after required assertions and cleanup succeed.

## Implementation notes for Codex

- Codex owns narrow implementation details within each ticket. It may choose the simplest repo-native design for helper functions, internal class/API shape, triangle/motion interpolation implementation, DDS/reset correlation, probe geometry, dependency organization, and cleanup mechanics, provided every stated contract, backward-compatibility requirement, bounded verifier, regression, Definition of Done, and exact PASS line succeeds.
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
- For this plan, “straight line parallel to the ground” means straight along a configured XY segment while maintaining constant terrain clearance above the mountain map: `z_world = terrain_height(x, y) + altitude_agl_m`.
- The validated target route must cover the full mission timeout. If the target reaches an endpoint, it reverses and continues along the same straight segment using `route_behavior: ping_pong`.
- Use MuJoCo cameras attached to each drone body for perspective views, published as ROS image topics and displayed in RViz2 through noVNC.
- RViz2 remains the only visualization UI; no external cloud dashboard is required.
- Active collision avoidance is out of scope for the MVP. The scripted target remains non-contacting/collision-filtered for deterministic execution. Unsafe trajectories are permitted to occur in simulation, but the E2E test fails any run where `actual_separation_m < safety.min_drone_separation_m` and reports the minimum observed separation.
- `/scenario/state` must remain observable to late subscribers through transient-local durability or periodic republishing of the current state. The implementation may use both; the verifier must include a late-subscriber check.

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
/scenario/state                              std_msgs/String, late-subscriber observable
/scenario/formation_error_m                  std_msgs/Float32
```

Scenario-state observability contract:

```text
/scenario/state publishes immediately on every state change
/scenario/state also republishes the current state at >= 1 Hz
/scenario/state uses transient-local durability if the repo-native ROS 2 publisher path supports it
late subscribers must observe the current state within 2 seconds
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

## Scenario lifecycle, completion, and safety contracts

Target route behavior:

```text
route_behavior: ping_pong
```

- The configured route is a straight XY line segment.
- The target starts at the configured sky pose and travels along the segment at `target_drone.speed_mps`.
- The validated first leg duration must be greater than or equal to `safety.mission_timeout_s + 10.0` so the route covers the full automated mission with buffer.
- When the target reaches either route endpoint, it reverses direction along the same segment and continues flying terrain-parallel.
- Each leg is straight and terrain-parallel; endpoint reversals are deterministic and keep the target inside the configured route segment.
- Target yaw follows the current direction of travel, including the 180° reversal at an endpoint.
- Endpoint reversal is unit-tested using simulated timestamps that reach and cross the endpoint. The short live probe in SCN-004 verifies first-leg straight-flight behavior only.
- The target continues this ping-pong motion after mission completion until the stack is shut down.

Mission completion and threshold-loss contract:

- `formation_error_m <= safety.formation_error_threshold_m` for `safety.formation_hold_required_s` continuous seconds triggers `COMPLETE`.
- `FORMATION_HOLD` begins once `formation_error_m` first falls within `safety.formation_error_threshold_m`.
- If `formation_error_m` rises above `safety.formation_error_threshold_m` before the required hold interval completes, the continuous-hold timer resets to `0.0`; the controller returns from `FORMATION_HOLD` to `INTERCEPT` on the next controller tick unless another timeout, health, geofence, clearance, or separation rule triggers `ABORT`.
- `COMPLETE` is allowed only after a fresh uninterrupted hold interval of `safety.formation_hold_required_s`.
- After `COMPLETE`, the chaser controller keeps issuing bounded formation-hold commands rather than disarming, landing, stopping abruptly, or directly commanding motors.
- The target continues its deterministic ping-pong route after `COMPLETE`.
- The end-to-end test expects final `/scenario/state == COMPLETE`, final formation error within threshold, final terrain clearance within threshold after clearance enforcement is active, final drone separation above threshold, and the stack down after cleanup.

Terrain-clearance enforcement phases:

- The chaser intentionally starts near the ground at `chaser_drone.initial_agl_m`; this may be below `safety.min_terrain_clearance_m`.
- Ground/low-clearance operation is permitted during `IDLE`, `HEALTH_CHECK`, `ARM`, and the initial low-altitude part of `TAKEOFF`.
- Terrain clearance is measured from startup, but clearance below `safety.min_terrain_clearance_m` must not trigger `ABORT` until enforcement is active.
- Clearance enforcement becomes active once the chaser first reaches `safety.min_terrain_clearance_m` AGL, and must be active no later than entering `INTERCEPT`.
- If the controller would enter `INTERCEPT` while the chaser is still below `safety.min_terrain_clearance_m` AGL, that is an unsafe-clearance failure and must `ABORT` instead of silently continuing.
- After enforcement becomes active, any sampled `terrain_clearance_m < safety.min_terrain_clearance_m` triggers `ABORT` and fails the E2E test.
- E2E terrain-clearance assertions use only samples at or after the same enforcement activation time, while still reporting minimum pre-activation and post-activation clearances for diagnosis.
- Target-drone terrain clearance is always enforced because the target starts in the sky.

Collision and separation policy:

- Active collision avoidance is out of scope for the MVP.
- The target remains non-contacting/collision-filtered for deterministic execution.
- Unsafe trajectories are permitted to occur in simulation; they are not automatically corrected by a collision-avoidance planner.
- The E2E test must fail any run where `actual_separation_m < safety.min_drone_separation_m`.
- The E2E test must report minimum observed separation for diagnosis on both success and failure.

## Make target ownership

```text
up-two-drone-viz       placeholder in SCN-000, real implementation in SCN-007
test-e2e-two-drone     placeholder in SCN-000, real implementation in SCN-009
```

`generate-map`, `test-map`, and `up-map-viz` remain owned by the `MAP-*` plan. Do not reassign them here.

Live-command cleanup rule:

- SCN-005 through SCN-008 must use bounded wrapper scripts or equivalent Make mechanics that run `make down` before startup and register cleanup on success, failure, and interruption.
- A failed live probe must not leave SITL, physics, ROS 2, RViz/noVNC, or helper containers running.
- Each live wrapper must print exactly one final `PASS:` line only after all assertions and cleanup succeed.
- For SCN-007, run `scripts/run_scn007_viz_check.sh` first for automated startup/probe/cleanup, then run `make up-two-drone-viz` separately only for browser-based manual viewing.
- Exploratory manual viewing must end with `make down`.

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

- Add a README section linking to `plans/two_drone_mountain_scenario_plan.md`.
- If `AGENTS.md` routes Codex to the active plan, update it to point to this plan only for the two-drone work.
- Add placeholder Make targets that fail clearly until implemented:
  - `up-two-drone-viz`
  - `test-e2e-two-drone`
- Placeholder output must say exactly which later SCN ticket implements the real target.
- Placeholder failures must be intentional and testable, not `command not found` or missing-target failures.
- Add both targets to `.PHONY` and `make help` with placeholder wording.
- Do not claim the two-drone scenario is runnable yet.
- Do not require host `python3`; run the verifier through Docker or a portable shell/Make check.

**Manual testing guidance**

Commands to run:

```bash
docker compose config
make help
make up-two-drone-viz || true
make test-e2e-two-drone || true
docker compose run --rm physics python3 scripts/verify_scn000_placeholders.py
make down
```

Expected result: both placeholder targets exist, fail intentionally with clear SCN ownership messages, and the verifier passes.

Cleanup command: `make down`.

Failure logs to inspect: `make help`, placeholder target output, and `docker compose config` output.

**Definition of Done**

```bash
test -f plans/two_drone_mountain_scenario_plan.md
docker compose config
make help
docker compose run --rm physics python3 scripts/verify_scn000_placeholders.py
```

The verifier must prove:

```text
up-two-drone-viz exists, fails intentionally, and mentions SCN-007
test-e2e-two-drone exists, fails intentionally, and mentions SCN-009
README links plans/two_drone_mountain_scenario_plan.md
no SCN ticket has been implemented yet
```

Done when the verifier exits `0` and its final line is exactly:

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
  start_xy_m: [-110.0, 0.0]
  heading_deg: 0.0
  speed_mps: 1.8
  altitude_agl_m: 35.0
  route_length_m: 240.0
  route_behavior: ping_pong

chaser_drone:
  name: chaser
  start_xy_m: [-120.0, -25.0]
  heading_deg: 0.0
  initial_agl_m: 0.3
  takeoff_agl_m: 12.0
  formation_offset_target_body_m: [-8.0, -5.0, 0.0]

safety:
  mission_timeout_s: 120.0
  formation_error_threshold_m: 5.0
  formation_hold_required_s: 10.0
  min_terrain_clearance_m: 5.0
  min_drone_separation_m: 3.0
  max_chaser_speed_mps: 12.0
  geofence_margin_m: 5.0

visualization:
  camera_rate_hz: 5.0
  marker_rate_hz: 5.0
```

Validation requirements:

- Reject unknown keys and non-finite numeric values.
- Require `name == mountain_two_drones`, `frame_id == world`, and `map.name == mountain_map`.
- Require `target_drone.route_behavior == ping_pong`.
- Require positive target speed, route length, mission timeout, formation threshold, hold duration, camera rate, and marker rate.
- Require nonnegative AGL values where applicable.
- Allow `chaser_drone.initial_agl_m` to be below `safety.min_terrain_clearance_m`; clearance enforcement follows the lifecycle gate defined above.
- Require chaser takeoff AGL greater than initial AGL.
- Normalize heading modulo `360` using the shared convention: `0° = +X`, positive counter-clockwise around `+Z`.
- Test heading conversion for `0°`, `90°`, `-90°`, `180°`, and `450°`.
- Require both spawn points and the full target route to remain inside the MAP geofence margin.
- Require `target_drone.route_length_m / target_drone.speed_mps >= safety.mission_timeout_s + 10.0`.
- Require the target route to maintain at least `safety.min_terrain_clearance_m` over the terrain.
- Require the configured formation offset norm to be at least `safety.min_drone_separation_m`.

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_config.py
docker compose run --rm physics python3 scripts/verify_two_drone_config.py --scenario mountain_two_drones
make down
```

Expected result: the verifier prints resolved target/chaser spawn poses, route endpoints, route duration, mission timeout, route behavior, normalized headings, and the terrain-clearance enforcement note for the low chaser start.

Cleanup command: `make down`.

Failure logs to inspect: config validation errors, terrain/geofence validation output, and the resolved route summary.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_config.py
docker compose run --rm physics python3 scripts/verify_two_drone_config.py --scenario mountain_two_drones
```

Done when both commands exit `0` and the verifier's final line is exactly:

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
- Configure the target drone as non-contacting/collision-filtered so it cannot destabilize the chaser; terrain contact for the chaser must still work.
- Preserve the existing default/landing-pad scenarios and map scenario behavior.

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_scenarios.py tests/test_two_drone_world.py
docker compose run --rm physics python3 scripts/verify_two_drone_world.py --scenario mountain_two_drones
make down
```

Expected result: the mountain map loads with exactly one target body and one chaser body; existing landing-pad and mountain-map scenarios still load.

Cleanup command: `make down`.

Failure logs to inspect: MuJoCo XML/model compile errors, scenario-loader logs, body/geom name reports, and terrain-contact assertions.

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
target collision policy is explicit and non-contacting
chaser-terrain contact still works
landing_pad and mountain_map scenarios still load
```

Done when both commands exit `0` and the verifier's final line is exactly:

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

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_motor_mixing.py tests/test_quad_attitude_response.py tests/test_mujoco_quad.py
docker compose run --rm physics python3 scripts/verify_chaser_dynamics.py
SCENARIO=landing_pad make test-e2e
make down
```

Expected result: the chaser shows tested vertical, roll, pitch, and yaw authority while the original landing-pad E2E still passes.

Cleanup command: `make down`.

Failure logs to inspect: motor-order diagnostics, angular acceleration assertions, SITL logs, and DFD E2E output.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_motor_mixing.py tests/test_quad_attitude_response.py tests/test_mujoco_quad.py
docker compose run --rm physics python3 scripts/verify_chaser_dynamics.py
SCENARIO=landing_pad make test-e2e
```

Done when all commands exit `0` and the verifier's final line is exactly:

```text
PASS: SCN-003 chaser dynamics support lateral, yaw, and vertical control
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
target route duration covers the full mission timeout plus a 10.0 second buffer
target reverses at each configured route endpoint using route_behavior=ping_pong
endpoint reversal is unit-tested with simulated timestamps that reach and cross an endpoint
the 15-second live probe verifies first-leg straight-flight behavior, not endpoint reversal
target publishes odometry and TF continuously during live simulation
```

Test thresholds:

```text
cross_track_error_m <= 0.25
speed_error_mps <= 0.25
AGL_error_m <= 0.50
heading_error_deg <= 2.0
minimum_terrain_clearance_m >= configured altitude_agl_m - 0.50
route_duration_s >= safety.mission_timeout_s + 10.0
route_endpoint_error_m <= 0.25
```

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_target_motion.py
docker compose run --rm physics python3 scripts/target_motion_probe.py --scenario mountain_two_drones --duration-s 15
make down
```

Expected result: unit tests prove deterministic endpoint reversal using simulated time, while the 15-second live probe proves first-leg straight terrain-parallel flight and confirms route duration covers the mission timeout plus a 10.0 second buffer.

Cleanup command: `make down`.

Failure logs to inspect: cross-track, speed, AGL, heading, simulated-time endpoint reversal, and route-duration diagnostics.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_target_motion.py
docker compose run --rm physics python3 scripts/target_motion_probe.py --scenario mountain_two_drones --duration-s 15
```

Done when both commands exit `0` and the probe's final line is exactly:

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
scripts/run_scn005_topic_check.sh
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

Marker and telemetry requirements:

- `/sim/scenario_markers` remains present and includes terrain/map scenario markers from MAP.
- `/sim/drone_markers` includes distinguishable target, chaser, target route, and desired formation-point markers.
- Static markers use transient-local delivery where appropriate.
- Odom topics use live volatile delivery.
- All marker points and odometry poses are finite.
- Marker frame IDs are `world`.
- Chaser alias topics match the chaser pose within `0.05 m` position tolerance and `1°` yaw tolerance.
- `scripts/run_scn005_topic_check.sh` owns live startup and cleanup: it runs `make down` before starting, installs EXIT/INT/TERM cleanup, starts only the required services, runs `two_drone_topic_probe.py` inside Docker, runs `make down` before returning control, and prints the exact SCN-005 `PASS:` line as its final line only after all assertions and cleanup succeed.
- The live probe must check late-subscriber transient-local delivery for markers and at least `5` live odometry messages for each drone.

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_ros_visualization.py tests/test_two_drone_topics.py
SCENARIO=mountain_two_drones bash scripts/run_scn005_topic_check.sh
```

Expected result: target/chaser odometry, TF, aliases, `/sim/scenario_markers`, and `/sim/drone_markers` are present; late marker subscribers receive static markers.

Cleanup command: the wrapper must run `make down` through a cleanup trap on success, failure, and interruption; running `make down` manually afterward must also be safe.

Failure logs to inspect: `docker compose logs physics ros2 sitl-json`, topic probe diagnostics, QoS reports, marker frame/bounds output, and wrapper cleanup output.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_ros_visualization.py tests/test_two_drone_topics.py
SCENARIO=mountain_two_drones bash scripts/run_scn005_topic_check.sh
```

Done when all commands exit `0`, the wrapper leaves the stack down, and the wrapper's final line is exactly:

```text
PASS: SCN-005 target and chaser telemetry, TF, aliases, and markers publish correctly
```

---

## SCN-006 — Add front camera perspectives for both drones

**Codex task**

Attach one front-facing MuJoCo camera to each drone and publish both image streams when `CAMERA_ENABLED=1`. Extend the existing `src/physics/ros_camera.py` camera path rather than creating a duplicate camera publishing abstraction; a new helper file is allowed only as a thin multi-camera coordinator that delegates to the existing camera implementation.

Files:

```text
models/scenarios/mountain_two_drones.xml
src/physics/ros_camera.py
scripts/two_drone_camera_probe.py
scripts/run_scn006_camera_check.sh
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

- Extend the existing `src/physics/ros_camera.py` path as the source of truth for camera publication.
- Do not create a second competing camera abstraction; if a helper/coordinator is introduced, it must be only a thin wrapper around `ros_camera.py` and must preserve the existing single-drone camera contract.
- Use `CAMERA_ENABLED=1` to enable image publication.
- Keep the default rate at `visualization.camera_rate_hz` from config.
- Keep image resolution low enough for Docker/noVNC performance.
- Publish valid `CameraInfo` for each camera.
- The target and chaser image streams must be nonblank and not byte-identical across at least `3` paired frames.
- Preserve the existing single-drone camera topic in non-two-drone scenarios.
- Tests must confirm the existing single-drone camera contract still passes when `SCENARIO` is not `mountain_two_drones`.
- `scripts/run_scn006_camera_check.sh` owns live startup and cleanup: it runs `make down` before starting, installs EXIT/INT/TERM cleanup, starts only the required services with `CAMERA_ENABLED=1`, runs the in-container camera probe, runs `make down` before returning control, and prints the exact SCN-006 `PASS:` line as its final line only after all assertions and cleanup succeed.

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_cameras.py
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn006_camera_check.sh
```

Expected result: both front camera image streams and camera info topics publish valid, nonblank, non-identical frames.

Cleanup command: the wrapper must run `make down` through a cleanup trap on success, failure, and interruption; running `make down` manually afterward must also be safe.

Failure logs to inspect: `docker compose logs physics ros2`, camera probe statistics, image encodings, frame IDs, render errors, and wrapper cleanup output.

**Definition of Done**

```bash
docker compose run --rm physics pytest -q tests/test_two_drone_cameras.py
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn006_camera_check.sh
```

Done when all commands exit `0`, the wrapper leaves the stack down, and the wrapper's final line is exactly:

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
scripts/run_scn007_viz_check.sh
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
- If bounded startup or probe checks fail, the wrapper must run `make down` before exiting nonzero.
- On successful `make up-two-drone-viz`, the target remains running for manual viewing until `make down`.
- `scripts/run_scn007_viz_check.sh` owns automated startup/probe/cleanup for test execution and must leave the stack down before printing the SCN-007 `PASS:` line.

**Manual testing guidance**

Commands to run:

```bash
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn007_viz_check.sh
# for extended manual viewing after the automated check passes:
CAMERA_ENABLED=1 make up-two-drone-viz
# open http://localhost:6080/vnc.html?autoconnect=true&resize=scale
make down
```

Expected result: RViz2 shows mountain terrain, target drone, chaser drone, route/formation markers, and two camera image panels through noVNC.

Cleanup command: the wrapper must run `make down` through a cleanup trap on success, failure, and interruption; for extended manual viewing started with `make up-two-drone-viz`, run `make down` manually afterward.

Failure logs to inspect: `docker compose logs viz ros2 physics`, `scripts/start_rviz_novnc.sh` output, RViz config path, noVNC container logs, and wrapper cleanup output.

**Definition of Done**

Acceptance sequence:

```bash
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn007_viz_check.sh
CAMERA_ENABLED=1 make up-two-drone-viz
# open http://localhost:6080/vnc.html?autoconnect=true&resize=scale
# confirm RViz2 shows mountain terrain, target drone, chaser drone, route/formation markers, and two image panels
make down
```

The automated wrapper must verify startup/probes and leave the stack down before printing its `PASS:` line. The second `make up-two-drone-viz` launch is explicitly for browser-based manual viewing.

Done when the automated wrapper exits `0`, failed automated checks clean up automatically, the second visualization launch is successful, the manual browser check is successful, `make down` leaves the stack stopped after manual viewing, and the wrapper's final line is exactly:

```text
PASS: SCN-007 two-drone RViz2/noVNC view works locally
```

---

## SCN-008 — Add chaser mission state machine

**Codex task**

Create a ROS 2 scenario controller that commands the ArduPilot-controlled chaser from ground start to intercept mode.

Files:

```text
src/ros2/drone_bridge/drone_bridge/scenario_controller.py
src/ros2/drone_bridge/setup.py
src/ros2/drone_bridge/package.xml
src/ros2/drone_bridge/launch/two_drone_scenario.launch.py
scripts/scenario_state_probe.py
scripts/run_scn008_state_check.sh
tests/test_scenario_controller.py
tests/test_scenario_state_qos.py
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

Package/launch requirements:

- Install `scenario_controller.py` as part of the existing `drone_bridge` Python package under `src/ros2/drone_bridge/drone_bridge/`.
- Update `src/ros2/drone_bridge/setup.py` and package metadata so the module is importable and runnable from inside Docker.
- Required console entry point: `scenario_controller = drone_bridge.scenario_controller:main`.
- Add or update a two-drone launch file, preferably `src/ros2/drone_bridge/launch/two_drone_scenario.launch.py`, so the controller starts with `SCENARIO=mountain_two_drones` through the existing Dockerized ROS 2 service.
- If the repo already has a canonical launch file for control nodes, update that file instead of adding an unused duplicate; the installed entry point remains required.
- `scripts/run_scn008_state_check.sh` owns live startup and cleanup: it runs `make down` before starting, installs EXIT/INT/TERM cleanup, starts the scenario stack, publishes `/scenario/start`, runs the in-container state probe, runs `make down` before returning control, and prints the exact SCN-008 `PASS:` line as its final line only after all assertions and cleanup succeed.

State machine:

```text
IDLE -> HEALTH_CHECK -> ARM -> TAKEOFF -> INTERCEPT -> FORMATION_HOLD -> COMPLETE
IDLE -> ... -> ABORT on timeout, health failure, geofence violation, unsafe clearance after clearance enforcement is active, or minimum-separation violation
```

Completion semantics:

```text
FORMATION_HOLD begins once formation_error_m first falls within safety.formation_error_threshold_m.
Ten continuous seconds with formation_error_m <= safety.formation_error_threshold_m triggers COMPLETE.
If formation_error_m rises above the threshold before the ten-second hold completes, the hold timer resets to zero and the controller returns to INTERCEPT on the next controller tick.
COMPLETE does not land or disarm the chaser; it keeps sending formation-hold commands until cleanup.
The target continues moving along the deterministic ping-pong route after COMPLETE until cleanup.
The E2E final state must be COMPLETE, with formation error within threshold and actual separation above min_drone_separation_m.
```

Requirements:

- Use the DFD pre-arm/EKF health gate before arming.
- Use finite timeouts for every state transition.
- Use altitude gain over the terrain/spawn pose, not absolute `z > threshold`, to confirm takeoff.
- Enforce geofence, maximum commanded speed, and minimum target separation throughout active flight.
- Enforce minimum terrain clearance only after the terrain-clearance enforcement window becomes active, matching the global terrain-clearance enforcement policy.
- Do not transition to `INTERCEPT` until the chaser is at or above `safety.min_terrain_clearance_m` AGL.
- Publish `/scenario/state` changes immediately and keep the latest state observable to late subscribers using transient-local durability or periodic republishing.
- Include a late-subscriber test for `/scenario/state`.
- Include a unit test where `FORMATION_HOLD` is entered, formation error rises above threshold, the hold timer resets to zero, and the controller returns to `INTERCEPT` while commands stay bounded.
- Do not command direct motors from ROS; use the existing ArduPilot-guided command path.

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm ros2 pytest -q tests/test_scenario_controller.py tests/test_scenario_state_qos.py
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn008_state_check.sh
```

Expected result: the controller reaches `INTERCEPT`, altitude gain is measured relative to terrain/spawn, the ground-start clearance exemption is honored, clearance enforcement becomes active only after the defined threshold/window, and a late subscriber can observe `/scenario/state`.

Cleanup command: the wrapper must run `make down` through a cleanup trap on success, failure, and interruption; running `make down` manually afterward must also be safe.

Failure logs to inspect: `docker compose logs ros2 physics sitl-json`, state transition logs, health-gate output, terrain-clearance enforcement diagnostics, scenario-state probe diagnostics, and wrapper cleanup output.

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_scenario_controller.py tests/test_scenario_state_qos.py
CAMERA_ENABLED=1 SCENARIO=mountain_two_drones bash scripts/run_scn008_state_check.sh
```

Done when all commands exit `0`, the wrapper leaves the stack down, and the wrapper's final line is exactly:

```text
PASS: SCN-008 chaser mission state machine reaches INTERCEPT safely
```

---

## SCN-009 — Implement intercept, formation hold, and e2e test

**Codex task**

Implement the guidance behavior and replace the placeholder `test-e2e-two-drone` with a real bounded end-to-end test.

Files:

```text
src/ros2/drone_bridge/drone_bridge/formation_guidance.py
src/ros2/drone_bridge/setup.py
src/ros2/drone_bridge/package.xml
src/ros2/drone_bridge/launch/two_drone_scenario.launch.py
scripts/formation_probe.py
scripts/e2e_mountain_two_drone.py
Makefile
README.md
tests/test_formation_guidance.py
```

Package/launch requirements:

- Install `formation_guidance.py` as part of the existing `drone_bridge` Python package under `src/ros2/drone_bridge/drone_bridge/`.
- Update `src/ros2/drone_bridge/setup.py`, package metadata, and launch wiring as needed so the controller and guidance modules run from installed package entry points inside Docker.
- Do not place ROS controller/guidance modules one directory above `drone_bridge/drone_bridge/`.

Behavior contract:

```text
during INTERCEPT, command the chaser toward the target's desired formation point
during FORMATION_HOLD, keep the chaser beside the target using formation_offset_target_body_m
ten continuous seconds with formation_error_m <= safety.formation_error_threshold_m triggers COMPLETE
if formation_error_m rises above the threshold before completion, reset the hold timer to zero and return to INTERCEPT on the next controller tick
after COMPLETE, the chaser continues formation-hold commands rather than landing, disarming, stopping abruptly, or commanding motors directly
after COMPLETE, the target continues its deterministic ping-pong terrain-parallel route until cleanup
chaser remains inside geofence and, after terrain-clearance enforcement becomes active, above minimum terrain clearance
active collision avoidance is out of scope; unsafe trajectories may occur in simulation, but the test fails on separation violation
```

Required metrics:

```text
formation_error_m             distance from chaser to desired formation point
actual_separation_m           distance from chaser to target
terrain_clearance_m           chaser_z - terrain_height(chaser_x, chaser_y)
clearance_enforced            true after first reaching min_terrain_clearance_m AGL or entering INTERCEPT
target_cross_track_error_m    target deviation from configured route
target_AGL_error_m            target terrain-relative altitude error
chaser_altitude_gain_m        chaser_z - initial_spawn_z
minimum_observed_separation_m minimum actual_separation_m over the run
```

End-to-end acceptance thresholds:

```text
target_cross_track_error_m <= 0.50
target_AGL_error_m <= 0.75
chaser_altitude_gain_m >= 5.0
formation_error_m <= safety.formation_error_threshold_m for at least safety.formation_hold_required_s continuous seconds
actual_separation_m >= safety.min_drone_separation_m at every sampled instant
terrain_clearance_m >= safety.min_terrain_clearance_m for every sample where clearance_enforced is true
mission finishes within safety.mission_timeout_s
final observed /scenario/state == COMPLETE
final formation_error_m <= safety.formation_error_threshold_m
minimum observed actual_separation_m is printed for diagnosis
```

`tests/test_formation_guidance.py` must cover threshold-loss behavior: entering `FORMATION_HOLD`, resetting the hold timer when error rises above threshold, returning to `INTERCEPT`, reacquiring the threshold, re-entering `FORMATION_HOLD`, and reaching `COMPLETE` only after a fresh continuous hold interval.

The live E2E test does not need to observe threshold loss during a naturally successful mission. Threshold-loss and reacquisition coverage remains in `tests/test_formation_guidance.py` unless a future ticket adds a deterministic bounded disturbance.

`test-e2e-two-drone` behavior:

- Runs `make down` before starting.
- Starts with `SCENARIO=mountain_two_drones` and honors `CAMERA_ENABLED`.
- Waits with bounded timeouts for telemetry, markers, `/scenario/state`, and optional cameras.
- Publishes `/scenario/start`.
- Verifies target route, chaser takeoff, ground-start clearance exemption, clearance enforcement window, decreasing formation error, ten continuous seconds of formation hold, final `COMPLETE` state, and minimum observed separation.
- Does not require a live threshold-loss/reacquisition event; threshold-loss, timer reset, and reacquisition coverage remain in `tests/test_formation_guidance.py` unless a later plan adds a bounded deterministic disturbance.
- Cleans up on success, failure, and interruption.
- Ends with the stack down.

Mandatory regressions in this ticket:

```bash
SCENARIO=landing_pad make test-e2e
make test-map
```

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm ros2 pytest -q tests/test_formation_guidance.py tests/test_scenario_controller.py
CAMERA_ENABLED=1 make test-e2e-two-drone
SCENARIO=landing_pad make test-e2e
make test-map
make down
```

Expected result: the mission reaches `COMPLETE` after ten continuous seconds within formation threshold, target route remains valid, the chaser honors the ground-start clearance exemption and stays clear of terrain after enforcement is active, and minimum observed drone separation is reported and safe.

Cleanup command: `make down`. The `test-e2e-two-drone` target must also clean up on success, failure, and interruption.

Failure logs to inspect: E2E timeline, `/scenario/state` history, formation/separation metrics, terrain-clearance enforcement window metrics, unit-test threshold-loss diagnostics, and `docker compose logs ros2 physics sitl-json`.

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_formation_guidance.py tests/test_scenario_controller.py
CAMERA_ENABLED=1 make test-e2e-two-drone
SCENARIO=landing_pad make test-e2e
make test-map
make down
```

Done when all commands exit `0`, the stack is down afterward, and the E2E test's final line is exactly:

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
src/ros2/drone_bridge/drone_bridge/multi_vehicle_bridge.py
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
- Use the live-stack cleanup contract for the optional live probe.

Suggested profile:

```bash
docker compose --profile dual-sitl up -d sitl-target sitl-chaser physics ros2 viz
```

**Manual testing guidance**

Commands to run:

```bash
docker compose run --rm ros2 pytest -q tests/test_dual_sitl_routing.py
bash scripts/run_dual_sitl_check.sh
make down
```

Expected result: optional dual-SITL mode shows separate heartbeats, separate vehicle IDs, separate JSON physics channels, and separate odometry streams.

Cleanup command: `make down`.

Failure logs to inspect: `docker compose logs sitl-target sitl-chaser physics ros2`, MAVLink port assignments, `SYSID_THISMAV` values, and routing probe output.

**Definition of Done**

```bash
docker compose run --rm ros2 pytest -q tests/test_dual_sitl_routing.py
bash scripts/run_dual_sitl_check.sh
```

Done when both vehicles have separate heartbeats, separate vehicle IDs, separate odometry streams, the stack is down afterward, and the probe's final line is exactly:

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
