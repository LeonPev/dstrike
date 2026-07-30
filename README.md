# dsriker

Docker-only two-drone mountain simulation for macOS. The main workflow runs a
scripted target along a terrain-following route while an ArduPilot-controlled
chaser takes off, intercepts it, and holds formation. MuJoCo, ROS 2, ArduPilot
SITL, RViz2, and noVNC all run inside Docker.

No host ROS 2, Python packages, ArduPilot, MuJoCo, or RViz installation is
required. This repository is simulation-only and does not command real
hardware.

The detailed behavior and safety contract is in the
[two-drone mountain scenario plan](plans/two_drone_mountain_scenario_plan.md).

## Requirements

- macOS with Docker Desktop
- Docker Engine and the Docker Compose plugin supplied by Docker Desktop
- Make, Git, a shell, and a browser

## Setup

```bash
test -f .env || cp .env.example .env
make preflight
make build
```

## Simulation modes

| Mode | Target | Chaser | Start behavior |
| --- | --- | --- | --- |
| Main chase (`DUAL_SITL=0`) | Deterministic scripted route | ArduPilot SITL | Target moves immediately; chaser waits for `/scenario/start` |
| Dual-SITL target route | ArduPilot SITL | ArduPilot SITL, stationary | Target route starts automatically; no `/scenario/start` subscriber |

The Make targets below force `SCENARIO=mountain_two_drones` and the appropriate
profile.

## Run the target-and-chaser mission

Start the browser-viewable stack:

```bash
SCENARIO=mountain_two_drones DUAL_SITL=0 CAMERA_ENABLED=1 make up-two-drone-viz
```

Wait for the command to print the ready message, then open:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Start the chase from another terminal:

```bash
docker compose exec -T ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 topic pub --once /scenario/start std_msgs/msg/Bool '{data: true}'"
```

The target is already following its configured terrain-parallel ping-pong
route. After the start command, the chaser progresses through health checks,
arming, takeoff, intercept, and formation hold. The launch pad and chaser begin
together at `x=-120 m, y=-25 m`.

Stop and remove the stack when finished:

```bash
make down
```

## Verify the complete chase

Run the bounded end-to-end mission:

```bash
DUAL_SITL=0 CAMERA_ENABLED=1 make test-e2e-two-drone
```

The check starts the stack, triggers the mission, verifies target motion,
chaser takeoff and intercept, terrain clearance, safe separation, and ten
continuous seconds of formation hold. It then shuts the stack down.

Expected final line:

```text
PASS: SCN-009 target straight terrain-parallel flight, chaser takeoff, intercept, and formation hold verified
```

## Dual-SITL target flight

The optional dual-SITL profile runs separate ArduPilot instances for the target
and chaser. In the current implementation, only the target route controller is
started: the target flies automatically and the chaser intentionally remains
on its pad.

This profile has no `/scenario/start` subscriber. Publishing the chase start
command while `up-dual-sitl` is running will wait indefinitely and will not
move the chaser.

Run the full target flight and endpoint-reversal check:

```bash
make test-dual-sitl-target
```

Start the browser-viewable dual-SITL target route:

```bash
CAMERA_ENABLED=1 make up-dual-sitl
```

Open the same noVNC URL, confirm the target holds position during startup and
then follows and reverses along the marked route, and clean up with:

```bash
make down
```

Expected automated-check final line:

```text
PASS: SCN-011 dual-SITL target follows the configured terrain-parallel ping-pong route
```

## Configuration

Mission geometry, route behavior, formation offset, timeouts, clearance, and
separation limits are defined in:

```text
config/scenarios/mountain_two_drones.yaml
```

Common environment settings are defined in `.env`:

| Variable | Default | Purpose |
| --- | --- | --- |
| `NOVNC_PORT` | `6080` | Browser viewer port bound to localhost |
| `VNC_GEOMETRY` | `1440x900x24` | RViz desktop size |
| `CAMERA_ENABLED` | `true` | Enable both simulated front cameras |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `480` | Camera image size |
| `CAMERA_FPS` | `10` | Camera publication rate |
| `ROS_DOMAIN_ID` | `42` | ROS 2 domain inside Docker |
| `ARDUPILOT_REF` | Pinned in `.env.example` | ArduPilot revision built into the image |

## Troubleshooting

If the chase publisher repeatedly prints `Waiting for at least 1 matching
subscription(s)...`, the dual-SITL profile is running. Stop it with Ctrl-C,
then run the `up-two-drone-viz` command from the main workflow above.

Inspect the running services and focused logs with:

```bash
docker compose ps
docker compose logs --tail 250 sitl-json physics ros2 viz
```

For dual-SITL target problems, use:

```bash
docker compose logs --tail 300 sitl-target sitl-chaser physics ros2 viz
make test-dual-sitl-target
```

Mission state and vehicle odometry can be inspected inside the ROS 2 container:

```bash
docker compose exec -T ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 topic echo /scenario/state"

docker compose exec -T ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 topic echo --once /sim/chaser/ground_truth/odom"
```

Flight logs and test diagnostics are written under `artifacts/`. Always finish
manual investigation with:

```bash
make down
```
