# Full Docker Setup Plan: ArduPilot + MuJoCo CPU + ROS 2 + RViz2/noVNC on macOS

Goal: run the full drone simulation stack with Docker Desktop for Mac only. No host installs are allowed except Docker Desktop, its Docker Engine, and the Docker Compose plugin. Keep the first version headless and browser-viewable.

Core architecture:

```text
Docker Compose network
  sitl       ArduCopter SITL, MAVLink, JSON external-physics mode
  physics    MuJoCo CPU physics backend, ArduPilot JSON UDP interface, sim/camera topics
  ros2       ROS 2 Humble workspace, MAVLink bridge/control nodes
  viz        RViz2 desktop exposed locally through noVNC
  test       smoke/e2e test runner
```

Manual viewing target:

```text
Browser on Mac -> noVNC -> RViz2 desktop -> http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Use one ticket at a time with Codex. Do not move to the next ticket until the Definition of Done passes manually.

---

## DFD-001 — Create repository scaffold

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
config/
```

Add Make targets:

```text
help, build, up, up-viz, down, logs, shell-ros2, shell-sitl, shell-physics, test, test-e2e
```

**Definition of Done**

```bash
cp .env.example .env
make help
docker compose config
```

Done when `docker compose config` exits successfully and no command requires ROS, Python, ArduPilot, or MuJoCo on macOS.

---

## DFD-002 — Build the base Docker image

**Codex task**
Implement `docker/Dockerfile` using `ros:humble-ros-base`.

Install container-side dependencies only:

```text
git, build-essential, cmake, ninja-build, ccache, gawk, wget, curl, jq
python3-colcon-common-extensions, python3-rosdep, python3-vcstool
ros-humble-nav-msgs, ros-humble-sensor-msgs, ros-humble-geometry-msgs
ros-humble-visualization-msgs, ros-humble-tf2-ros, ros-humble-rosgraph-msgs
ros-humble-rviz2, ros-humble-image-transport, ros-humble-tf2-tools
xvfb, x11vnc, novnc, websockify, fluxbox, dbus-x11
libxkbcommon-x11-0, libxcb-xinerama0
libgl1, libgl1-mesa-dri, mesa-utils, libglfw3, libosmesa6, libegl1
Python pip packages: mujoco, numpy, pymavlink, pytest
```

Set:

```bash
MUJOCO_GL=osmesa
ROS_DOMAIN_ID=42
PYTHONUNBUFFERED=1
LIBGL_ALWAYS_SOFTWARE=1
```

**Definition of Done**

```bash
make build
docker compose run --rm ros2 bash -lc 'python3 -c "import mujoco, rclpy, pymavlink; print(\"ok\")"'
docker compose run --rm ros2 bash -lc 'command -v rviz2 && test -f /usr/share/novnc/vnc.html && command -v websockify && echo rviz-novnc-ok'
```

Done when Python imports work and RViz2/noVNC commands exist inside Docker.

---

## DFD-003 — Pin and build ArduPilot SITL inside Docker

**Codex task**
Add an ArduPilot source/build step to the image or a named Docker volume.

Requirements:

- Add `ARDUPILOT_REF` to `.env.example`.
- Do not use floating `master` in final form.
- Clone ArduPilot with submodules.
- Run the Ubuntu prerequisite script inside the container.
- Build Copter SITL with `waf`.

Expected build flow inside Docker:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git /opt/ardupilot
cd /opt/ardupilot
git checkout "$ARDUPILOT_REF"
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

**Definition of Done**

```bash
docker compose run --rm sitl bash -lc '/opt/ardupilot/build/sitl/bin/arducopter --help | head'
docker compose run --rm sitl bash -lc 'test -x /opt/ardupilot/Tools/autotest/sim_vehicle.py && echo sim_vehicle-ok'
```

Done when the `arducopter` binary exists and `sim_vehicle.py` is executable inside Docker.

---

## DFD-004 — Start baseline ArduCopter SITL with built-in physics

**Codex task**
Add a `baseline` Compose profile that starts ArduCopter in normal quad SITL mode before introducing MuJoCo.

Requirements:

- Expose MAVLink to the Compose network.
- Prefer endpoint `udp:ros2:14550`.
- Add `scripts/mavlink_wait_heartbeat.py` using `pymavlink`.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile baseline up sitl-baseline ros2
```

Terminal 2:

```bash
docker compose exec ros2 python3 scripts/mavlink_wait_heartbeat.py --connect udp:0.0.0.0:14550
```

Done when the script prints a heartbeat with `sysid`, `compid`, vehicle type, and autopilot type.

---

## DFD-005 — Add the ArduPilot JSON protocol stub

**Codex task**
Create a `physics-stub` service that implements only the ArduPilot JSON external-physics protocol.

Requirements:

- Listen on UDP port `9002`.
- Parse ArduPilot's binary PWM packet.
- Log `magic`, `frame_rate`, `frame_count`, and first 4 PWM channels.
- Reply with a static valid JSON physics state containing:
  - `timestamp`
  - `imu.gyro`
  - `imu.accel_body`
  - `position`
  - `velocity`
  - `attitude` or `quaternion`
- Start SITL in JSON model mode pointed at the physics service.

**Definition of Done**

```bash
docker compose --profile json-stub up sitl-json physics-stub
```

Done when `physics-stub` logs increasing `frame_count` values and SITL logs show valid JSON input.

---

## DFD-006 — Create a minimal MuJoCo quadrotor model

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

**Definition of Done**

```bash
docker compose run --rm physics python3 -m src.physics.self_test
```

Done when the self-test proves both cases:

```text
zero thrust: altitude decreases
hover-ish thrust: altitude remains bounded or increases slowly
```

---

## DFD-007 — Connect MuJoCo to ArduPilot JSON SITL

**Codex task**
Replace `physics-stub` with the real `physics` service.

Requirements:

- Receive PWM frames from ArduPilot.
- Convert PWM channels 1-4 to rotor thrusts.
- Step MuJoCo at a stable fixed timestep.
- Send ArduPilot JSON state using NED convention:
  - `position`: north, east, down
  - `velocity`: north, east, down
  - `imu.gyro`: body rad/s
  - `imu.accel_body`: body m/s²
  - `attitude` or `quaternion`
- Include unit tests for ENU/z-up to NED conversion.

**Definition of Done**

```bash
docker compose --profile mujoco up sitl-json physics
```

Done when the system runs for 60 seconds with increasing frame counts, no physics-process crash, and SITL logs show valid JSON input.

---

## DFD-008 — Add ROS 2 MAVLink bridge node

**Codex task**
Create a ROS 2 Python package under `src/ros2/drone_bridge`.

Minimum node behavior:

- Connect to ArduPilot MAVLink on UDP `14550`.
- Publish heartbeat/status on `/drone/heartbeat`.
- Publish local odometry on `/drone/odom` using `nav_msgs/Odometry`.
- Subscribe to:
  - `/drone/arm` as `std_msgs/Bool`
  - `/drone/takeoff_alt_m` as `std_msgs/Float32`
  - `/drone/cmd_vel` as `geometry_msgs/Twist`
- Convert those topics to MAVLink arm, takeoff, and GUIDED velocity commands.

**Definition of Done**

Terminal 1:

```bash
docker compose --profile mujoco up sitl-json physics ros2
```

Terminal 2:

```bash
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 node list'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/heartbeat --once'
```

Done when `/drone/heartbeat` publishes from SITL through the ROS 2 node.

---

## DFD-009 — Command arm and takeoff from ROS 2

**Codex task**
Add a simple ROS 2 command path and test script.

Expected manual commands:

```bash
ros2 topic pub --once /drone/arm std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /drone/takeoff_alt_m std_msgs/msg/Float32 '{data: 2.0}'
```

The bridge should place ArduPilot into a suitable guided mode before takeoff if needed.

**Definition of Done**

```bash
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic pub --once /drone/arm std_msgs/msg/Bool "{data: true}"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic pub --once /drone/takeoff_alt_m std_msgs/msg/Float32 "{data: 2.0}"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/odom --once'
```

Done when odometry altitude begins increasing after the takeoff command.

---

## DFD-010 — Publish simulation state for visualization

**Codex task**
Make the `physics` service also act as a ROS 2 publisher.

Publish these topics:

```text
/sim/clock optional rosgraph_msgs/Clock
/sim/ground_truth/odom nav_msgs/Odometry
/tf world -> base_link
/sim/scenario_markers visualization_msgs/MarkerArray
```

Requirements:

- Publish `world`, `base_link`, and rotor/site frames.
- Publish simple markers for landing pad, obstacles, gates, and scenario boundaries.
- Keep `/drone/odom` from the MAVLink bridge separate from `/sim/ground_truth/odom`.

**Definition of Done**

```bash
docker compose --profile mujoco up sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic list | grep -E "/sim/ground_truth/odom|/tf|/sim/scenario_markers"'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /sim/ground_truth/odom --once'
```

Done when ROS 2 receives live ground-truth odometry, TF, and scenario markers.

---

## DFD-011 — Add MuJoCo camera image publishing

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
- Make image resolution configurable in `.env`.
- Default to a low rate such as 5-10 Hz.
- Continue physics stepping even if camera publishing is disabled.

**Definition of Done**

```bash
docker compose --profile mujoco up sitl-json physics ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic hz /sim/camera/front/image_raw'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /sim/camera/front/camera_info --once'
```

Done when image frames publish at a stable rate and the physics process keeps running.

---

## DFD-012 — Add local RViz2 + noVNC browser visualization service

**Codex task**
Add a `viz` service that runs RViz2 inside Docker and exposes the desktop through noVNC.

Required files:

```text
scripts/start_rviz_novnc.sh
config/drone.rviz
```

Startup script requirements:

- Source `/opt/ros/humble/setup.bash`.
- Source `/workspace/install/setup.bash` if it exists.
- Start `Xvfb` on display `:1`.
- Start a lightweight window manager such as `fluxbox`.
- Start `x11vnc` on internal port `5900` with no password; expose only noVNC bound to localhost through Compose.
- Start `websockify`/noVNC on `0.0.0.0:6080` inside the container; bind it to `127.0.0.1:6080` on the Mac host.
- Launch `rviz2 -d /workspace/config/drone.rviz`.
- Use software OpenGL: `LIBGL_ALWAYS_SOFTWARE=1`.

Compose requirements:

```yaml
viz:
  build:
    context: .
    dockerfile: docker/Dockerfile
  command: bash -lc "scripts/start_rviz_novnc.sh"
  ports:
    - "127.0.0.1:6080:6080"
  environment:
    ROS_DOMAIN_ID: "42"
    DISPLAY: ":1"
    LIBGL_ALWAYS_SOFTWARE: "1"
  depends_on:
    - ros2
    - physics
```

Add:

```bash
make up-viz
```

Expected manual browser connection:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

`config/drone.rviz` should load displays for:

```text
Fixed Frame: world
TF: /tf
Odometry: /drone/odom
Odometry: /sim/ground_truth/odom
MarkerArray: /sim/scenario_markers
Image: /sim/camera/front/image_raw
```

**Definition of Done**

```bash
make up-viz
docker compose logs viz | grep -Ei "rviz|vnc|websockify"
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic list | grep -E "/drone/odom|/sim/ground_truth/odom|/tf|/sim/scenario_markers"'
```

Done when `http://localhost:6080/vnc.html?autoconnect=true&resize=scale` opens locally in the Mac browser, shows an RViz2 desktop, and RViz2 displays live drone pose/TF/scenario markers.

---

## DFD-013 — Add scenario files and manual visual checks

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
- Publish matching scenario markers to RViz2.
- Add a README section called `Viewing scenarios`.

**Definition of Done**

```bash
SCENARIO=landing_pad make up-viz
SCENARIO=obstacle_course make up-viz
```

Done when changing `SCENARIO` changes both the MuJoCo world and the markers visible in RViz2.

---

## DFD-014 — Add end-to-end takeoff smoke test

**Codex task**
Create `tests/e2e_takeoff.py` or `scripts/e2e_takeoff.py`.

Test behavior:

1. Wait for MAVLink heartbeat.
2. Wait for ROS 2 bridge topics.
3. Arm.
4. Command 2 m takeoff.
5. Monitor `/drone/odom`.
6. Pass if altitude exceeds 1 m within a fixed timeout.
7. Send land/disarm cleanup if possible.

Add Make target:

```bash
make test-e2e
```

**Definition of Done**

```bash
make test-e2e
```

Done when the test exits `0` and prints:

```text
E2E PASS: heartbeat, arm, takeoff, odom altitude threshold reached
```

---

## DFD-015 — Add observability and artifacts

**Codex task**
Add enough telemetry to debug without GUI tools.

Requirements:

- Add `make topic-list`, `make topic-hz`, `make logs`, and `make bag-record`.
- Store ArduPilot logs, ROS bags, and test logs under `artifacts/` mounted from the host repo.
- Record MCAP if available; otherwise record default ROS 2 bag format.

**Definition of Done**

```bash
make up-viz
make topic-list
make topic-hz TOPIC=/drone/odom
make bag-record DURATION=10
ls artifacts/
```

Done when `/drone/odom`, `/sim/ground_truth/odom`, `/tf`, `/sim/scenario_markers`, and optional `/sim/camera/front/image_raw` are visible, publish at stable rates, and logs/bags appear in `artifacts/`.

---

## DFD-016 — Document the one-command workflow

**Codex task**
Update `README.md` with the final happy path.

Required commands:

```bash
cp .env.example .env
make build
make up-viz
make test-e2e
make down
```

Document:

- No host ROS/ArduPilot/MuJoCo install required.
- Docker is still required on macOS.
- Browser visualization uses RViz2 through noVNC on port `6080`.
- Expected services and ports.
- Common failure checks:
  - no heartbeat
  - no JSON frames
  - EKF unhealthy
  - ROS 2 node not visible
  - RViz2 opens but no topics are visible
  - camera topic missing
  - Docker CPU or memory too constrained

**Definition of Done**

From a fresh clone on macOS with only Docker available:

```bash
cp .env.example .env
make build
make up-viz
make test-e2e
make down
```

Done when the documented path completes without hidden manual steps and the browser shows live simulation topics.

---

## Optional later ticket — Native ArduPilot DDS ROS 2 path

**ID: DFD-017**

Add ArduPilot's native AP_DDS / micro-ROS-Agent integration after the MAVLink bridge works.

**Definition of Done**

```bash
ros2 node list
ros2 node info /ap
ros2 topic echo /ap/geopose/filtered --once
```

Done when ArduPilot publishes native ROS 2 topics through DDS inside Docker.

---

## Source references for implementation

- ArduPilot JSON external physics interface: https://ardupilot.org/dev/docs/sitl-with-JSON.html
- ArduPilot JSON protocol details: https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md
- ArduPilot ROS 2 install notes: https://ardupilot.org/dev/docs/ros2-install.html
- ArduPilot ROS 2 SITL notes: https://ardupilot.org/dev/docs/ros2-sitl.html
- ROS Docker image: https://hub.docker.com/_/ros/
- RViz2 source/docs: https://github.com/ros2/rviz
- noVNC source/docs: https://github.com/novnc/noVNC
- MuJoCo Python/offscreen rendering docs: https://mujoco.readthedocs.io/en/stable/python.html
