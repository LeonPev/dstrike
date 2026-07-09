# Cleanup Plan Completed

The abandoned DFD-012 browser-viewer implementation was rolled back by starting
the replacement branch from `dfd-011-mujoco-camera-publishing`.

Current replacement branch:

```text
dfd-012-rviz-novnc-visualization
```

The local visualization path is now:

```text
Browser on Mac -> noVNC -> RViz2 desktop in Docker -> ROS 2 topics
```

Manual viewer URL:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Replacement files:

```text
scripts/start_rviz_novnc.sh
config/drone.rviz
```

Verification checklist:

```bash
make build
make up-viz
docker compose logs viz | grep -Ei "rviz|vnc|websockify|xvfb|fluxbox"
docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && ros2 topic list | grep -E "/drone/odom|/sim/ground_truth/odom|/tf|/sim/scenario_markers|/sim/drone_markers|/sim/camera/front/image_raw"'
git diff --check
```

Manual pass line:

```text
PASS: DFD-012 RViz2 noVNC browser visualization passed
```
