# dstrike

Docker Desktop for Mac drone simulation stack. It runs ArduPilot SITL, MuJoCo
physics, a ROS 2 MAVLink bridge, and RViz2/noVNC inside Docker.

No host ROS 2, ArduPilot, MuJoCo, RViz2, Python package, or alternate container
runtime install is required.

## Requirements

- macOS with Docker Desktop for Mac
- Docker Engine and Docker Compose plugin bundled with Docker Desktop
- Make, a shell, and a browser

## Quick Start

```bash
cp .env.example .env
make build
make up-viz
```

Open the browser viewer:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Stop everything:

```bash
make down
```

## Smoke Test

Run the full headless takeoff test:

```bash
make test-e2e
```

Expected success line:

```text
E2E PASS: heartbeat, arm, takeoff, odom altitude threshold reached
```

`make test-e2e` resets the Compose stack, starts the MuJoCo/SITL/ROS 2 services,
runs the takeoff test, and shuts the stack down. Run `make up-viz` again when
you want the browser viewer.

## What Runs

| Service | Purpose | Ports |
| --- | --- | --- |
| `sitl-json` | ArduCopter SITL in JSON external-physics mode | Internal UDP to `physics:9002` and `ros2:14550` |
| `physics` | MuJoCo physics, scenario markers, ground-truth odometry, camera publishing | Internal UDP `9002` |
| `ros2` | ROS 2 MAVLink bridge and command topics | Internal UDP `14550` |
| `viz` | RViz2 desktop streamed through noVNC | Host `127.0.0.1:6080` to container `6080/tcp` |

Only noVNC is published to the Mac host by default.

## Expected Topics

```text
/drone/heartbeat
/drone/odom
/sim/ground_truth/odom
/tf
/sim/scenario_markers
/sim/drone_markers
/sim/camera/front/image_raw
/sim/camera/front/camera_info
```

Check the live topic set:

```bash
make topic-list
```

## Common Commands

```bash
make help
make preflight
make build
make up-viz
make test-e2e
make topic-list
make topic-hz TOPIC=/drone/odom
make bag-record DURATION=10
make logs LOG_FLAGS=
make down
```

Artifacts are written under:

```text
artifacts/ardupilot/
artifacts/rosbags/
artifacts/test-logs/
```

## Configuration

Edit `.env` after copying `.env.example`.

| Variable | Default | Meaning |
| --- | --- | --- |
| `NOVNC_PORT` | `6080` | Browser viewer port on `127.0.0.1` |
| `SCENARIO` | `landing_pad` | Scenario: `empty_world`, `landing_pad`, or `obstacle_course` |
| `CAMERA_ENABLED` | `true` | Publish the MuJoCo front camera topics |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `640` / `480` | Camera image size |
| `CAMERA_FPS` | `10` | Camera publish rate |
| `ROS_DOMAIN_ID` | `42` | ROS 2 domain used inside containers |
| `ARDUPILOT_REF` | pinned SHA in `.env.example` | ArduPilot commit baked into the image |

Restart after config changes:

```bash
make down
make up-viz
```

## Troubleshooting

Browser does not open:

```bash
docker compose ps
docker compose logs viz
```

RViz2 opens but has no data:

```bash
make topic-list
docker compose logs ros2 physics viz
```

No heartbeat or takeoff fails:

```bash
docker compose logs sitl-json ros2
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic echo /drone/heartbeat --once'
```

No JSON physics frames:

```bash
docker compose logs sitl-json physics
```

Camera topic missing:

```bash
grep -E "CAMERA_ENABLED|CAMERA_TOPIC|CAMERA_NAME" .env
docker compose logs physics
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/image_probe.py /sim/camera/front/image_raw'
```

Docker feels too slow or unstable:

```bash
make preflight
docker stats
```

Increase Docker Desktop CPU or memory limits, then restart the stack.
