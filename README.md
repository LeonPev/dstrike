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
