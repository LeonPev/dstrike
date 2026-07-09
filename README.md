# dstrike

Headless, browser-viewable drone simulation stack for macOS using Docker
Desktop for Mac.

## Host Docker Preflight

DFD-000 verifies only host-side Docker Desktop availability. Do not install ROS,
Python, ArduPilot, MuJoCo, or alternate Docker-compatible runtimes on macOS for
this project.

Required host software:

- Docker Desktop for Mac
- Docker Engine included with Docker Desktop
- Docker Compose plugin included with Docker Desktop
- Make and the macOS shell

Run the manual checks:

```bash
docker version
docker compose version
docker run --rm hello-world
docker system df
make preflight
```

`make preflight` reports the Mac architecture, Docker Desktop / Docker Engine
details, Docker Compose version, Docker CPU and memory limits when Docker
exposes them, and Docker disk usage from `docker system df`. It also runs the
`hello-world` Linux container smoke test.

Success ends with:

```text
PASS: DFD-000 Docker host preflight passed
```

## Repository Scaffold

DFD-001 adds the Docker Compose and directory layout for the headless simulation
stack. Host-side project commands stay limited to Docker Desktop, Docker
Compose, Make, the shell, and a browser.

Repository layout:

```text
compose.yaml
.env.example
Makefile
docker/Dockerfile
scripts/
src/physics/
src/ros2/
models/
models/scenarios/
tests/
artifacts/
```

Create local defaults before running Compose:

```bash
cp .env.example .env
```

Common commands:

```bash
make help
make build
make up
make up-viz
make down
make test
```

The placeholder services use one shared image, mount this repository at
`/workspace`, and set `/workspace` as the container working directory. Later
tickets replace the placeholder commands with ArduPilot SITL, MuJoCo physics,
ROS 2 bridge/control nodes, and Foxglove visualization.

DFD-001 manual checks:

```bash
cp .env.example .env
make help
docker compose config
docker compose run --rm ros2 pwd
docker compose run --rm ros2 test -f /workspace/README.md
echo "PASS: DFD-001 scaffold and compose config passed"
```

## Base Docker Image

DFD-002 builds the shared simulation image from `ros:humble-ros-base`. All ROS,
MuJoCo, MAVLink, build, and render dependencies are installed inside Docker.

DFD-002 manual checks:

```bash
make build
docker compose run --rm ros2 bash -lc 'python3 -c "import mujoco, rclpy, pymavlink; print(\"ok\")"'
docker compose run --rm ros2 bash -lc 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml --show-args >/dev/null && echo foxglove-ok'
docker compose run --rm physics python3 -m src.physics.render_smoke
```

Success ends with:

```text
PASS: DFD-002 base image imports and headless render passed
```

## ArduPilot SITL Image Build

DFD-003 pins ArduPilot to a full commit SHA and bakes Copter SITL into the
shared Docker image at `/opt/ardupilot`. The default pin is:

```text
92b0cd788ec29406f26c6f9c31d5ceedbd1cc538
```

The Docker build clones ArduPilot with submodules, checks out the pinned commit,
runs the Ubuntu prerequisite installer, builds Copter SITL with `waf`, and
writes the resolved image build ref to `/opt/ardupilot/ARDUPILOT_BUILD_REF`.

DFD-003 manual checks:

```bash
make build
docker compose run --rm sitl bash -lc 'git -C /opt/ardupilot rev-parse HEAD'
docker compose run --rm sitl bash -lc 'cat /opt/ardupilot/ARDUPILOT_BUILD_REF'
docker compose run --rm sitl bash -lc 'test "$(git -C /opt/ardupilot rev-parse HEAD)" = "$(cat /opt/ardupilot/ARDUPILOT_BUILD_REF)"'
docker compose run --rm sitl bash -lc '/opt/ardupilot/build/sitl/bin/arducopter --help | head'
docker compose run --rm sitl bash -lc 'test -x /opt/ardupilot/Tools/autotest/sim_vehicle.py && echo sim_vehicle-ok'
echo "PASS: DFD-003 pinned ArduPilot SITL build passed"
```

## Baseline ArduCopter SITL

DFD-004 adds a `baseline` Compose profile that starts ArduCopter SITL in normal
quad mode with ArduPilot's built-in physics. The explicit MAVLink route is:

```text
sitl-baseline -> udp:ros2:14550
```

The service keeps the planned `sim_vehicle.py ... --no-mavproxy
--out=udp:ros2:14550` shape and passes `--serial0=udpclient:ros2:14550`
directly to the SITL binary so MAVLink still flows without MAVProxy.

Start the baseline stack in one terminal:

```bash
docker compose --profile baseline up sitl-baseline ros2
```

Wait for the heartbeat from another terminal:

```bash
docker compose logs sitl-baseline | grep -E "14550|MAVProxy|udp|serial0|out"
docker compose exec ros2 python3 scripts/mavlink_wait_heartbeat.py --connect udp:0.0.0.0:14550
```

The heartbeat script prints `sysid`, `compid`, vehicle type, autopilot type,
base mode, custom mode, and the ticket PASS line:

```text
PASS: DFD-004 baseline SITL heartbeat passed
```

## ArduPilot JSON Protocol Parser

DFD-005 added the binary PWM parser and the original static JSON protocol
stub. DFD-007 replaces the Compose runtime with the real MuJoCo-backed
`physics` service, but the parser and focused unit tests remain useful.

The parser decodes ArduPilot's binary PWM packet, supports the 16-channel magic
`18458`, detects the 32-channel magic `29569`, and keeps the mandatory JSON
field shape used by the live physics service:

```text
timestamp
imu.gyro
imu.accel_body
position
velocity
quaternion
```

Run the focused parser unit check:

```bash
make test-json-stub
```

## Minimal MuJoCo Quadrotor Model

DFD-006 adds `models/quad.xml`, a no-GUI MuJoCo model with one free
`base_link` body, placeholder mass and inertia, four rotor force application
sites, and a fixed `front_cam` camera. The Python wrapper in
`src/physics/mujoco_quad.py` loads the model, resets it, applies four motor
thrust values, steps MuJoCo, and returns pose, velocity, angular velocity, and
body acceleration.

Run the self-test inside Docker:

```bash
docker compose run --rm physics python3 -m src.physics.self_test
```

The self-test proves zero thrust falls and hover-ish thrust remains bounded,
then prints:

```text
PASS: DFD-006 MuJoCo quad self-test passed
```

## MuJoCo JSON SITL Integration

DFD-007 connects ArduCopter SITL's JSON external-physics mode to the real
MuJoCo quad model:

```text
sitl-json -> resolves physics, then starts --model JSON:<physics-ip>
physics -> UDP 9002 MuJoCo JSON external-physics endpoint
```

The physics service receives ArduPilot PWM frames, maps PWM channels 1-4 into
rotor thrust, advances MuJoCo with bounded fixed substeps, and replies with
newline-terminated JSON containing NED `position`/`velocity`, FRD body
`imu.gyro`/`imu.accel_body`, and scalar-first `quaternion`. It intentionally
does not send Euler `attitude`.

Run the focused convention test:

```bash
docker compose run --rm physics pytest tests/test_frame_conventions.py
```

Run the integration stack in one terminal:

```bash
docker compose --profile mujoco up sitl-json physics
```

Check the logs from another terminal:

```bash
docker compose logs physics | grep -E "frame_count|timestep|pwm|thrust|ned"
docker compose logs sitl-json | grep -A8 "JSON received"
echo "PASS: DFD-007 MuJoCo JSON integration passed"
```

## ROS 2 MAVLink Bridge

DFD-008 adds the `drone_bridge` ROS 2 Python package. The `ros2` Compose
service builds the workspace, starts `drone_mavlink_bridge`, listens for
ArduPilot MAVLink on UDP `14550`, publishes status on `/drone/heartbeat`, and
publishes MAVLink `LOCAL_POSITION_NED` as ROS ENU/z-up odometry on
`/drone/odom`.

Start the integration stack in one terminal:

```bash
docker compose --profile mujoco up sitl-json physics ros2
```

Check the ROS bridge from another terminal:

```bash
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 node list'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/heartbeat --once'
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/odom --once'
docker compose exec ros2 bash -lc 'cd /workspace && pytest tests/test_ros_frame_conventions.py'
echo "PASS: DFD-008 ROS 2 MAVLink bridge passed"
```
