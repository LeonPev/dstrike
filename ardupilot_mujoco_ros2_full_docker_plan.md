# Full Docker Setup Plan: ArduPilot + MuJoCo CPU + ROS 2 + Browser Visualization on macOS

Goal: run the full drone simulation stack with Docker Desktop for Mac only. No host installs are allowed except Docker Desktop, its Docker Engine, and the Docker Compose plugin. Keep the first version headless and browser-viewable.

Core architecture:

```text
Docker Compose network
  sitl       ArduCopter SITL, MAVLink, JSON external-physics mode
  physics    MuJoCo CPU physics backend, ArduPilot JSON UDP interface, sim/camera topics
  ros2       ROS 2 Humble workspace, MAVLink bridge/control nodes
  viz        Foxglove Bridge WebSocket for browser visualization
  test       smoke/e2e test runner
```

Manual viewing target:

```text
Browser on Mac -> Foxglove WebSocket -> ws://localhost:8765
```

Plan-wide conventions:

```text
Host installs allowed: Docker Desktop for Mac only.
No alternate Docker-compatible runtimes are supported or documented.
Canonical in-container repo/workspace path: /workspace.
ArduPilot JSON and MAVLink convention: NED, down-positive altitude.
ROS 2 topics and TF convention: ENU / z-up unless explicitly named otherwise.
```

Implementation decisions fixed by this plan:

```text
Container platform: Docker Desktop for Mac, Docker Engine, and Docker Compose plugin.
ArduPilot build location: baked into the Docker image at /opt/ardupilot.
ArduPilot ref format: full commit SHA, not branch name and not floating tag.
SITL launcher: sim_vehicle.py for both baseline and JSON modes.
ArduPilot JSON attitude field: quaternion only.
ROS message timestamps: wall-clock/node time for v1; no /clock topic in the first version.
E2E runner location: scripts/e2e_takeoff.py.
ROS bag format: default ROS 2 sqlite3 bag format.
Host-side tooling: Docker Desktop, Docker Compose, Make, shell, and browser only.
Native ArduPilot DDS: optional later ticket only, not part of the happy path.
```

Use one ticket at a time with Codex. Do not move to the next ticket until the Definition of Done passes manually.

Every ticket must end with a manual pass signal in this exact shape:

```text
PASS: DFD-XXX <short human-readable result>
```

If a ticket adds a helper script, the helper script should print that `PASS` line itself. If a ticket only adds configuration or docs, the final manual command can be `echo "PASS: ..."` after the listed checks succeed.

---

## DFD-000 - Host Docker preflight

**Codex task**
Create a short `README.md` preflight section and a `make preflight` target that checks only host-side Docker Desktop availability. Do not require ROS, Python, ArduPilot, MuJoCo, or any non-Docker tooling on macOS.

Manual facts to capture:

```text
Mac architecture
Docker Desktop / Docker Engine version
Docker Compose version
Docker CPU and memory limits if visible
Free Docker disk space if visible
```

**Definition of Done**

```bash
docker version
docker compose version
docker run --rm hello-world
docker system df
make preflight
```

Done when Docker Desktop can run Linux containers and `make preflight` prints the detected Docker details.

Manual PASS signal:

```text
PASS: DFD-000 Docker host preflight passed
```

---

## DFD-001 - Create repository scaffold

**Codex task**
Create the repo layout and minimal developer commands.

Expected files:

```text
compose.yaml
.env.example
Makefile
README.md
docker/Dockerfile
scripts/
src/physics/
src/ros2/
models/
models/scenarios/
tests/
artifacts/
```

Add Make targets:

```text
help, preflight, build, up, up-viz, down, logs, shell-ros2, shell-sitl, shell-physics, test, test-e2e
```

Requirements:

- Mount the repository at `/workspace` in every service.
- Keep host-side commands limited to Docker, Docker Compose, Make, shell, and the browser.
- Add `.gitkeep` files for intentionally empty directories.
- Add `.env.example` defaults for `ROS_DOMAIN_ID`, `ARDUPILOT_REF`, `SCENARIO`, camera settings, and optional CPU/resource settings.

**Definition of Done**

```bash
cp .env.example .env
make help
docker compose config
docker compose run --rm ros2 pwd
docker compose run --rm ros2 test -f /workspace/README.md
```

Done when `docker compose config` exits successfully, the container workdir is `/workspace`, and no command requires ROS, Python, ArduPilot, or MuJoCo on macOS.

Manual PASS signal:

```text
PASS: DFD-001 scaffold and compose config passed
```

---

## DFD-002 - Build the base Docker image

**Codex task**
Implement `docker/Dockerfile` using `ros:humble-ros-base`.

Install container-side dependencies only:

```text
git, build-essential, cmake, ninja-build, ccache, gawk, wget, curl, jq
python3-colcon-common-extensions, python3-rosdep, python3-vcstool
ros-humble-nav-msgs, ros-humble-sensor-msgs, ros-humble-geometry-msgs
ros-humble-visualization-msgs, ros-humble-tf2-ros, ros-humble-rosgraph-msgs
ros-humble-foxglove-bridge
libgl1, libglfw3, libosmesa6, libegl1
Python pip packages: mujoco, numpy, pymavlink, pytest
```

Set:

```bash
MUJOCO_GL=osmesa
ROS_DOMAIN_ID=42
PYTHONUNBUFFERED=1
```

Add:

```text
src/physics/render_smoke.py
```

`render_smoke.py` must create a MuJoCo `GLContext`, make it current, render a tiny offscreen frame, check that width/height are correct, and print pixel min/max so the result is manually inspectable without a GUI.

**Definition of Done**

```bash
make build
docker compose run --rm ros2 bash -lc 'python3 -c "import mujoco, rclpy, pymavlink; print(\"ok\")"'
docker compose run --rm ros2 bash -lc 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml --show-args >/dev/null && echo foxglove-ok'
docker compose run --rm physics python3 -m src.physics.render_smoke
```

Done when imports work, Foxglove Bridge launch arguments are visible inside Docker, and MuJoCo headless rendering produces a non-empty image buffer.

Manual PASS signal:

```text
PASS: DFD-002 base image imports and headless render passed
```

---

## DFD-003 - Pin and build ArduPilot SITL inside Docker

**Codex task**
Add an ArduPilot source/build step to the Docker image. Do not use a named Docker volume for ArduPilot source or build outputs in the first version.

Requirements:

- Add `ARDUPILOT_REF` to `.env.example`.
- Do not use floating `master`, `main`, branch names, or mutable tags in final form.
- Use a full ArduPilot commit SHA for `ARDUPILOT_REF`.
- Pass `ARDUPILOT_REF` as a Docker build argument and container environment variable.
- Clone ArduPilot with submodules.
- Run the Ubuntu prerequisite script inside the container.
- Build Copter SITL with `waf`.
- Write the resolved commit SHA to `/opt/ardupilot/ARDUPILOT_BUILD_REF`.
- Log the resolved ArduPilot commit during image build.

Expected build flow inside Docker:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git /opt/ardupilot
cd /opt/ardupilot
git checkout "$ARDUPILOT_REF"
git submodule update --init --recursive
git rev-parse HEAD | tee /opt/ardupilot/ARDUPILOT_BUILD_REF
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

**Definition of Done**

```bash
make build
docker compose run --rm sitl bash -lc 'git -C /opt/ardupilot rev-parse HEAD'
docker compose run --rm sitl bash -lc 'cat /opt/ardupilot/ARDUPILOT_BUILD_REF'
docker compose run --rm sitl bash -lc 'test "$(git -C /opt/ardupilot rev-parse HEAD)" = "$(cat /opt/ardupilot/ARDUPILOT_BUILD_REF)"'
docker compose run --rm sitl bash -lc '/opt/ardupilot/build/sitl/bin/arducopter --help | head'
docker compose run --rm sitl bash -lc 'test -x /opt/ardupilot/Tools/autotest/sim_vehicle.py && echo sim_vehicle-ok'
```

Done when the resolved commit is visible and stable, the `arducopter` binary exists, and `sim_vehicle.py` is executable inside Docker.

Manual PASS signal:

```text
PASS: DFD-003 pinned ArduPilot SITL build passed
```

---

## DFD-004 - Start baseline ArduCopter SITL with built-in physics

**Codex task**
Add a `baseline` Compose profile that starts ArduCopter in normal quad SITL mode before introducing MuJoCo.

Requirements:

- Expose MAVLink to the Compose network.
- Use one explicit MAVLink routing path from SITL to the `ros2` service.
- Use this `sim_vehicle.py` command shape:

```bash
cd /opt/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --no-mavproxy --out=udp:ros2:14550
```

- Add `scripts/mavlink_wait_heartbeat.py` using `pymavlink`.
- The heartbeat script must print `sysid`, `compid`, vehicle type, autopilot type, base mode, and custom mode.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile baseline up sitl-baseline ros2
```

Terminal 2:

```bash
docker compose logs sitl-baseline | grep -E "14550|MAVProxy|udp|serial0|out"
docker compose exec ros2 python3 scripts/mavlink_wait_heartbeat.py --connect udp:0.0.0.0:14550
```

Done when the script prints a heartbeat with `sysid`, `compid`, vehicle type, autopilot type, base mode, and custom mode.

Manual PASS signal:

```text
PASS: DFD-004 baseline SITL heartbeat passed
```

---

## DFD-005 - Add the ArduPilot JSON protocol stub

**Codex task**
Create a `physics-stub` service that implements only the ArduPilot JSON external-physics protocol.

Requirements:

- Listen on UDP port `9002`.
- Parse ArduPilot's binary PWM packet.
- Support 16-channel magic `18458`.
- Detect and log 32-channel magic `29569`, even if 32-channel mode is not implemented yet.
- Log `magic`, `frame_rate`, `frame_count`, and first 4 PWM channels.
- Reply to the exact sender IP/port that sent the PWM packet.
- Reply with newline-terminated JSON.
- Reply with a static valid JSON physics state containing:
  - `timestamp`
  - `imu.gyro`
  - `imu.accel_body`
  - `position`
  - `velocity`
  - `quaternion`
- Start SITL in JSON model mode pointed at the physics service:

```text
--model JSON:physics-stub
```

**Definition of Done**

```bash
docker compose --profile json-stub up sitl-json physics-stub
```

In another terminal:

```bash
docker compose logs physics-stub | grep -E "magic|frame_rate|frame_count|pwm"
docker compose logs sitl-json | grep -A8 "JSON received"
```

Done when `physics-stub` logs increasing `frame_count` values and SITL logs show all mandatory JSON fields were received.

Manual PASS signal:

```text
PASS: DFD-005 JSON protocol stub passed
```

---

## DFD-006 - Create a minimal MuJoCo quadrotor model

**Codex task**
Add `models/quad.xml` and a standalone MuJoCo self-test script.

Model requirements:

- One free body named `base_link`.
- Reasonable mass and inertia placeholders.
- Four rotor application sites.
- One fixed camera named `front_cam`.
- No GUI or viewer dependency.

Add `src/physics/mujoco_quad.py` with:

- load model
- reset
- step with 4 motor thrust values
- return pose, velocity, angular velocity, and body acceleration

Add `src/physics/self_test.py`.

**Definition of Done**

```bash
docker compose run --rm physics python3 -m src.physics.self_test
```

Done when the self-test proves both cases:

```text
zero thrust: altitude decreases
hover-ish thrust: altitude remains bounded or increases slowly
```

Manual PASS signal:

```text
PASS: DFD-006 MuJoCo quad self-test passed
```

---

## DFD-007 - Connect MuJoCo to ArduPilot JSON SITL

**Codex task**
Replace `physics-stub` with the real `physics` service.

Requirements:

- Receive PWM frames from ArduPilot.
- Convert PWM channels 1-4 to rotor thrusts.
- Step MuJoCo at a stable fixed timestep.
- Bound maximum internal substep size if ArduPilot frame timing changes.
- Send ArduPilot JSON state using NED convention:
  - `position`: north, east, down
  - `velocity`: north, east, down
  - `imu.gyro`: body rad/s
  - `imu.accel_body`: body m/s^2
- Send `quaternion` in ArduPilot's expected order and do not send Euler `attitude` in the first version.
- Include unit tests for ENU/z-up to NED conversion.
- Include unit tests that prove ROS z-up altitude increases while ArduPilot JSON down decreases.
- Log frame count, timestep, PWM range, thrust range, and NED position at a low rate such as 1 Hz.

**Definition of Done**

```bash
docker compose run --rm physics pytest tests/test_frame_conventions.py
docker compose --profile mujoco up sitl-json physics
```

In another terminal:

```bash
docker compose logs physics | grep -E "frame_count|timestep|pwm|thrust|ned"
docker compose logs sitl-json | grep -A8 "JSON received"
```

Done when the system runs for 60 seconds with increasing frame counts, no physics-process crash, and SITL logs show valid JSON input.

Manual PASS signal:

```text
PASS: DFD-007 MuJoCo JSON integration passed
```

---

## DFD-007A - Add physics sanity and EKF readiness checks

**Codex task**
Add a health-check script that makes the MuJoCo + JSON SITL path debuggable before adding ROS command logic.

Create:

```text
scripts/mavlink_health.py
```

The script must connect to MAVLink and print:

```text
heartbeat sysid/compid
mode
armed state
GPS/EKF/prearm text, or `unavailable` if no such message arrives before timeout
attitude
local position or altitude estimate, or `unavailable` if no such message arrives before timeout
```

Requirements:

- Keep the script read-only; it must not arm or change mode.
- Return non-zero if no heartbeat is received.
- Return non-zero if repeated MAVLink STATUSTEXT messages contain obvious fatal JSON, IMU, or EKF errors.
- Save recent health output under `artifacts/health/`.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile mujoco up sitl-json physics ros2
```

Terminal 2:

```bash
docker compose exec ros2 python3 scripts/mavlink_health.py --connect udp:0.0.0.0:14550
docker compose logs sitl-json | grep -Ei "EKF|prearm|JSON received|IMU"
ls artifacts/health/
```

Done when the script prints heartbeat, mode, armed state, sensor/EKF status, and exits successfully without commanding the vehicle.

Manual PASS signal:

```text
PASS: DFD-007A physics and EKF readiness passed
```

---

## DFD-008 - Add ROS 2 MAVLink bridge node

**Codex task**
Create a ROS 2 Python package under `src/ros2/drone_bridge`.

Minimum node behavior:

- Connect to ArduPilot MAVLink on UDP `14550`.
- Publish heartbeat/status on `/drone/heartbeat`.
- Publish local odometry on `/drone/odom` using `nav_msgs/Odometry`.
- Publish `/drone/odom` in ROS ENU / z-up convention.
- Subscribe to:
  - `/drone/arm` as `std_msgs/Bool`
  - `/drone/takeoff_alt_m` as `std_msgs/Float32`
  - `/drone/cmd_vel` as `geometry_msgs/Twist`
- Convert those topics to MAVLink arm, takeoff, and GUIDED velocity commands.
- Include unit tests for MAVLink/NED to ROS/ENU odometry conversion.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile mujoco up sitl-json physics ros2
```

Terminal 2:

```bash
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 node list'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/heartbeat --once'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/odom --once'
docker compose exec ros2 bash -lc 'cd /workspace && pytest tests/test_ros_frame_conventions.py'
```

Done when `/drone/heartbeat` and `/drone/odom` publish from SITL through the ROS 2 node and frame conversion tests pass.

Manual PASS signal:

```text
PASS: DFD-008 ROS 2 MAVLink bridge passed
```

---

## DFD-009 - Command arm and takeoff from ROS 2

**Codex task**
Add a simple ROS 2 command path and test script.

Expected manual commands:

```bash
ros2 topic pub --once /drone/arm std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /drone/takeoff_alt_m std_msgs/msg/Float32 '{data: 2.0}'
```

The bridge should place ArduPilot into a suitable guided mode before takeoff if needed.

Requirements:

- Add a read-only odometry watcher script such as `scripts/ros_wait_altitude.py`.
- The watcher must treat `/drone/odom.pose.pose.position.z` as ROS ENU z-up altitude.
- The watcher must print starting altitude, latest altitude, and delta.
- Add cleanup guidance for land/disarm, but do not hide cleanup errors.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile mujoco up sitl-json physics ros2
```

Terminal 2:

```bash
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic pub --once /drone/arm std_msgs/msg/Bool "{data: true}"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic pub --once /drone/takeoff_alt_m std_msgs/msg/Float32 "{data: 2.0}"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/ros_wait_altitude.py --topic /drone/odom --min-delta 0.5 --timeout 30'
```

Done when ROS z-up odometry altitude increases after the takeoff command.

Manual PASS signal:

```text
PASS: DFD-009 ROS arm and takeoff passed
```

---

## DFD-010 - Publish simulation state for visualization

**Codex task**
Make the `physics` service also act as a ROS 2 publisher.

Publish these topics:

```text
/sim/ground_truth/odom nav_msgs/Odometry
/tf world -> base_link
/sim/scenario_markers visualization_msgs/MarkerArray
```

Requirements:

- Publish `world`, `base_link`, and rotor/site frames.
- Publish ROS/TF data in ENU / z-up convention.
- Use wall-clock/node timestamps in the first version.
- Do not publish `/clock` or `/sim/clock` in the first version.
- Publish simple markers for landing pad, obstacles, gates, and scenario boundaries.
- Keep `/drone/odom` from the MAVLink bridge separate from `/sim/ground_truth/odom`.
- Include a small topic probe script that waits for expected topics and prints their message types.

**Definition of Done**

```bash
docker compose --profile mujoco up sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/ros_topic_probe.py /sim/ground_truth/odom /tf /sim/scenario_markers'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /sim/ground_truth/odom --once'
```

Done when ROS 2 receives live ground-truth odometry, TF, and scenario markers.

Manual PASS signal:

```text
PASS: DFD-010 ROS visualization state topics passed
```

---

## DFD-011 - Add MuJoCo camera image publishing

**Codex task**
Add offscreen MuJoCo rendering inside the `physics` service and publish images.

Publish:

```text
/sim/camera/front/image_raw sensor_msgs/Image
/sim/camera/front/camera_info sensor_msgs/CameraInfo
```

Requirements:

- Use the `front_cam` camera in `models/quad.xml`.
- Render headlessly with `MUJOCO_GL=osmesa`.
- Use a MuJoCo offscreen `GLContext` and call `make_current()` before rendering.
- Make image resolution configurable in `.env`.
- Default to a low rate such as 5-10 Hz.
- Continue physics stepping even if camera publishing is disabled.
- Add `scripts/image_probe.py` that prints width, height, encoding, fps estimate, and pixel min/max.

**Definition of Done**

```bash
docker compose --profile mujoco up sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic hz /sim/camera/front/image_raw'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /sim/camera/front/camera_info --once'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/image_probe.py /sim/camera/front/image_raw'
```

Done when image frames publish at a stable rate, the image probe reports non-empty image buffers, and the physics process keeps running.

Manual PASS signal:

```text
PASS: DFD-011 MuJoCo camera publishing passed
```

---

## DFD-012 - Add browser visualization service

**Codex task**
Add a `viz` service that runs Foxglove Bridge.

Compose requirements:

```yaml
viz:
  build:
    context: .
    dockerfile: docker/Dockerfile
  command: >
    bash -lc "source /opt/ros/humble/setup.bash &&
              if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi &&
              ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
  ports:
    - "8765:8765"
  environment:
    ROS_DOMAIN_ID: "42"
```

Add:

```bash
make up-viz
```

Expected manual browser connection:

```text
Foxglove -> Open connection -> Foxglove WebSocket -> ws://localhost:8765
```

**Definition of Done**

```bash
make up-viz
docker compose logs viz | grep -i foxglove
```

Done when a browser can connect to `ws://localhost:8765` and Foxglove shows these topics:

```text
/drone/odom
/sim/ground_truth/odom
/tf
/sim/scenario_markers
/sim/camera/front/image_raw
```

Manual PASS signal:

```text
PASS: DFD-012 Foxglove browser visualization passed
```

---

## DFD-013 - Add scenario files and manual visual checks

**Codex task**
Add scenario loading to the physics service.

Create:

```text
models/scenarios/empty_world.xml
models/scenarios/landing_pad.xml
models/scenarios/obstacle_course.xml
```

Requirements:

- Add `SCENARIO=landing_pad` to `.env.example`.
- Load scenario XML into MuJoCo at startup.
- Publish matching scenario markers to Foxglove.
- Print the selected scenario name and resolved XML path at physics startup.
- Add `scripts/scenario_probe.py` that checks the selected scenario and marker count.
- Add a README section called `Viewing scenarios`.

**Definition of Done**

```bash
SCENARIO=landing_pad make up-viz
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/scenario_probe.py --expect landing_pad'
SCENARIO=obstacle_course make up-viz
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/scenario_probe.py --expect obstacle_course'
```

Done when changing `SCENARIO` changes both the MuJoCo world and the markers visible in Foxglove.

Manual PASS signal:

```text
PASS: DFD-013 scenario switching passed
```

---

## DFD-014 - Add end-to-end takeoff smoke test

**Codex task**
Create `scripts/e2e_takeoff.py`.

Test behavior:

1. Wait for MAVLink heartbeat.
2. Wait for ROS 2 bridge topics.
3. Record starting ROS z-up altitude from `/drone/odom`.
4. Arm.
5. Command 2 m takeoff.
6. Monitor `/drone/odom`.
7. Pass if ROS z-up altitude delta exceeds 1 m within a fixed timeout.
8. Save a concise text log under `artifacts/e2e/`.
9. Send land/disarm cleanup if possible.

Add Make target:

```bash
make test-e2e
```

**Definition of Done**

```bash
make test-e2e
ls artifacts/e2e/
```

Done when the test exits `0` and prints:

```text
E2E PASS: heartbeat, arm, takeoff, odom altitude threshold reached
```

Manual PASS signal:

```text
PASS: DFD-014 end-to-end takeoff smoke test passed
```

---

## DFD-015 - Add observability and artifacts

**Codex task**
Add enough telemetry to debug without GUI tools.

Requirements:

- Add `make topic-list`, `make topic-hz`, `make logs`, and `make bag-record`.
- Store ArduPilot logs, ROS bags, health logs, image probe logs, and test logs under `artifacts/` mounted from the host repo.
- Record default ROS 2 sqlite3 bags. Do not add MCAP in the first version.
- Add `make doctor` that runs read-only checks for service status, expected topics, heartbeat, JSON frames, and artifact paths.

**Definition of Done**

```bash
make up-viz
make topic-list
make topic-hz TOPIC=/drone/odom
make bag-record DURATION=10
make doctor
ls artifacts/
```

Done when `/drone/odom`, `/sim/ground_truth/odom`, `/tf`, `/sim/scenario_markers`, and `/sim/camera/front/image_raw` are visible, publish at stable rates, and logs/bags appear in `artifacts/`.

Manual PASS signal:

```text
PASS: DFD-015 observability and artifacts passed
```

---

## DFD-016 - Document the one-command workflow

**Codex task**
Update `README.md` with the final happy path.

Required commands:

```bash
cp .env.example .env
make preflight
make build
make up-viz
make test-e2e
make down
```

Document:

- No host ROS/ArduPilot/MuJoCo install required.
- Docker Desktop for Mac is required.
- Alternate Docker-compatible runtimes are out of scope and should not be documented as supported choices.
- Browser visualization uses Foxglove WebSocket on port `8765`.
- Expected services and ports.
- Frame conventions:
  - ArduPilot JSON/MAVLink: NED, down-positive.
  - ROS 2 topics and TF: ENU / z-up.
- Expected PASS strings for each DFD.
- Common failure checks:
  - Docker Desktop unavailable
  - Docker CPU or memory too constrained
  - no heartbeat
  - no JSON frames
  - EKF unhealthy
  - ROS 2 node not visible
  - Foxglove connects but no topics
  - camera topic missing or blank
  - stale Compose services

**Definition of Done**

From a fresh clone on macOS with only Docker available:

```bash
cp .env.example .env
make preflight
make build
make up-viz
make test-e2e
make down
```

Done when the documented path completes without hidden manual steps and the browser shows live simulation topics.

Manual PASS signal:

```text
PASS: DFD-016 documented one-command workflow passed
```

---

## Optional later ticket - Native ArduPilot DDS ROS 2 path

**ID: DFD-017**

Add ArduPilot's native AP_DDS / micro-ROS-Agent integration after the MAVLink bridge works.

Requirements:

- Keep this separate from the MAVLink bridge path.
- Add `micro-ROS-Agent` only in this optional ticket.
- Document `DDS_ENABLE=1`.
- Keep `DDS_DOMAIN_ID` and `ROS_DOMAIN_ID` aligned.
- Do not remove the MAVLink bridge until the DDS path has its own e2e test.

**Definition of Done**

```bash
ros2 node list
ros2 node info /ap
ros2 topic echo /ap/geopose/filtered --once
```

Done when ArduPilot publishes native ROS 2 topics through DDS inside Docker.

Manual PASS signal:

```text
PASS: DFD-017 native ArduPilot DDS path passed
```

---

## Source references for implementation

- ArduPilot JSON external physics interface: https://ardupilot.org/dev/docs/sitl-with-JSON.html
- ArduPilot JSON protocol details: https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md
- ArduPilot Linux build setup: https://ardupilot.org/dev/docs/building-setup-linux.html
- ArduPilot SITL setup: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- ArduPilot SITL usage: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- ArduPilot ROS 2 install notes: https://ardupilot.org/dev/docs/ros2-install.html
- ArduPilot ROS 2 SITL notes: https://ardupilot.org/dev/docs/ros2-sitl.html
- ROS Docker image: https://hub.docker.com/_/ros/
- Foxglove ROS 2 visualization docs: https://docs.foxglove.dev/docs/getting-started/frameworks/ros2
- Foxglove Bridge docs: https://docs.foxglove.dev/docs/fleet/bridge
- MuJoCo Python/offscreen rendering docs: https://mujoco.readthedocs.io/en/stable/python.html
