# Project Status and Architecture Review

Snapshot date: 2026-07-28; repository-state refresh: 2026-07-29. This review distinguishes files and commits found in the repository from commands proved at runtime during this review.

## 1. Executive summary

The project is a Docker-only, local drone simulation stack intended for Docker Desktop on macOS.
ArduPilot SITL supplies the flight-controller behavior, with a pinned ArduPilot revision built into the shared image.
MuJoCo provides CPU physics and exchanges ArduPilot's external-physics JSON protocol over UDP.
ROS 2 Humble provides the MAVLink adapter, control/mission nodes, telemetry, TF, markers, and camera topics.
RViz2 runs inside Docker and is exposed only on localhost through noVNC for browser viewing.
The mountain-map plan adds deterministic procedural terrain, terrain-aware spawning, and static terrain/nature visualization without cloud services.
The two-drone plan was written as a future extension to the base stack, but the current history and files already contain SCN-000 through SCN-009 and optional SCN-010 implementations.
The repository contains optional dual-SITL routing, but a 2026-07-29 live run proved that dual mode has no target route-command source: the second target SITL is present but does not fly the scripted target's route.

## 2. Current repository state

| Item | Observed state |
| --- | --- |
| Reviewed implementation branch | `feature/SCN-010-dual-sitl` |
| Reviewed implementation tip | `6cd1f87015a77af380e68e9309d3e100c1ef97ab` — `Fix SCN-010 browser startup` |
| Upstream | The implementation branch now tracks `origin/feature/SCN-010-dual-sitl` and is synchronized with it |
| Remote divergence | `origin/main...feature/SCN-010-dual-sitl` is `0 51`; `origin/main` is still the initial commit |
| Pre-report working tree | Clean: no modified, deleted, staged, or untracked files |
| State after creating this report | Not clean only because `plans/project_status_architecture_review.md` is untracked |
| Modified tracked files | None |
| Deleted tracked files | None |
| Untracked files | `plans/project_status_architecture_review.md` only |
| Compose services at review time | None listed by the full-profile `docker compose ... ps --all` query |

There is no P0 dirty-tree or missing-plan blocker. Before another ticket starts, this report must be reviewed and committed coherently. The implementation branch is published, but `origin/main` remains stale; starting from `origin/main` would omit 51 commits. The local history is linear and shows MAP-006 as an ancestor of SCN-000 and SCN-009 as an ancestor of SCN-010.

## 3. Current architecture

```text
Docker Compose
  |-- sitl-json: default ArduPilot SITL in JSON external-physics mode
  |-- physics: MuJoCo CPU physics, JSON backend, and simulation ROS publishers
  |-- ros2: ROS 2 MAVLink adapter, scenario controller, and formation guidance
  |-- viz: RViz2 desktop through Xvfb/x11vnc/noVNC
  |-- sitl-baseline: optional built-in-physics baseline profile
  |-- sitl-target + sitl-chaser: optional SCN-010 dual-SITL profile
  `-- test: one-shot scaffold container, not a full aggregate test suite
```

Default data flow:

- `sitl-json` sends binary PWM packets to `physics`, and `physics` returns JSON state, over UDP on the Compose network (`physics:9002/udp`).
- SITL sends MAVLink to the `ros2` service on UDP `14550`; the ROS adapter publishes heartbeat/odometry and translates ROS command topics back to MAVLink. The optional dual profile adds separate target/chaser JSON and MAVLink routes (`9003` and `14551` for the target path).
- `physics` publishes ground-truth odometry, TF, scenario/drone/terrain/nature markers, and optional camera images as ROS topics.
- RViz2 consumes those ROS topics in the `world` frame.
- A local browser connects to `127.0.0.1:6080`, then noVNC streams the containerized RViz2 desktop.
- No external cloud visualization or map provider is required.

The reusable hardware boundary is the ROS-facing mission/control contract: simulation state is published as ROS telemetry, while `src/ros2/drone_bridge/drone_bridge/mavlink_bridge.py` and the launch files adapt those commands to ArduPilot. No real-hardware execution is implemented or authorized.

## 4. Important conventions and contracts

| Contract | Current convention/evidence |
| --- | --- |
| Workflow | Robotics dependencies, generation, and tests run in Docker. No host ROS 2, MuJoCo, ArduPilot, or project Python environment is required. |
| Host requirements | Docker Desktop/Engine/Compose, `make`, Git, a shell, and a browser only. |
| Container workdir | `/workspace`, configured in `compose.yaml` and `docker/Dockerfile`. |
| Coordinates | MuJoCo and ROS use an ENU `world` frame with z up; the ArduPilot JSON boundary uses NED/FRD with z down. Conversions are centralized in `src/physics/frame_conventions.py`. |
| Camera switch | `CAMERA_ENABLED`; default `true` in `.env.example`/Compose. Do not use the stale `ENABLE_CAMERAS` spelling found once in the base plan. |
| Scenario switch | `SCENARIO`; default `landing_pad`. Canonical loading is through `src/physics/scenarios.py`. |
| Reset switch | `SIM_RESET_AFTER_FRAMES`; present in `compose.yaml` and `src/physics/mujoco_json.py`, with `0` meaning disabled. It is not currently documented in `.env.example` or the README configuration table. |
| ROS domain | `ROS_DOMAIN_ID=42` by default inside containers. |
| PASS output | MAP/SCN verifiers are intended to print exactly one final `PASS:` line only after assertions and required cleanup succeed. DFD-014 retains its legacy `E2E PASS:` text. Static exceptions are listed in section 9. |
| Visualization | RViz2/noVNC is the only UI and is bound to localhost. |
| Hardware safety | Simulation only. Real-hardware commands and tests require a separate adapter, explicit approval, and documented safety checks. |

## 5. Implemented base stack status

“Confirmed from repo” below means the implementation and supporting files are present; it does not mean the live behavior passed during this review.

| Base item | Status | Evidence | Runtime conclusion |
| --- | --- | --- | --- |
| Docker image and services | Confirmed from repo | `docker/Dockerfile`, `docker/requirements.txt`, `compose.yaml`; `docker compose config --quiet` exited 0 | Configuration parses; image build/start not run |
| Make targets | Confirmed from repo | `Makefile`; `make help` exited 0 and listed base, map, SCN, and cleanup targets | Help surface verified only |
| ArduPilot SITL startup | Partially confirmed | Pinned `ARDUPILOT_REF` in `.env.example`/Dockerfile; `sitl-baseline`, `sitl-json`, `sitl-target`, and `sitl-chaser` in `compose.yaml`; `scripts/mavlink_wait_heartbeat.py` | No heartbeat or startup test run |
| MuJoCo CPU physics | Confirmed from repo | `src/physics/mujoco_quad.py`, `src/physics/mujoco_json.py`, `src/physics/multi_vehicle_json.py`, `models/quad.xml`, scenario XML files, and physics tests | No live JSON/physics loop run |
| ROS 2 topics and probes | Confirmed from repo | `src/physics/ros_telemetry.py`, `src/physics/ros_visualization.py`, `src/ros2/drone_bridge/`, and `scripts/ros_*_probe.py` | Topic discovery/rates not run |
| RViz2/noVNC | Partially confirmed | `viz` service in `compose.yaml`, `scripts/start_rviz_novnc.sh`, `config/drone.rviz`, `config/mountain_map.rviz`, `config/two_drone_mountain.rviz` | Browser/manual visualization not run |
| End-to-end tests | Confirmed from repo | `Makefile` targets `test-e2e` and `test-e2e-two-drone`; `scripts/e2e_takeoff.py`, `scripts/e2e_mountain_two_drone.py`, and bounded wrappers | Neither E2E was run |
| Cleanup behavior | Partially confirmed | `make down`; EXIT/INT/TERM traps in map/SCN wrappers; current full-profile Compose query listed no services | Cleanup paths were not exercised, and some wrappers suppress cleanup failure or print PASS too early |

`make test` is only the Compose scaffold check (`README.md` exists); it is not an aggregate unit/regression target. The base flight regression remains `SCENARIO=landing_pad make test-e2e`.

## 6. Active plans and intended order

The canonical historical implementation order is:

1. Base Docker/RViz2 stack.
2. Mountain terrain map through MAP-006.
3. Two-drone mountain scenario through SCN-009.
4. Optional MAP-007 and/or SCN-010 follow-ups.

Repository history follows that order and has advanced through optional SCN-010. Only optional MAP-007 was not found as implemented.

| Plan | Purpose and starting prerequisite | Next ticket now | Readiness and blockers |
| --- | --- | --- | --- |
| `plans/ardupilot_mujoco_ros2_full_docker_plan.md` | Establish the Docker-only SITL/MuJoCo/ROS/RViz base; starts from the repository scaffold and Docker host gate | Historically MAP-000; already implemented | Base implementation/history is present through DFD-016, but the plan labels tickets `DCK-*`; live regression not rerun |
| `plans/mountain_terrain_map_plan.md` | Add deterministic local mountain physics/visuals after the completed base stack | Optional MAP-007 only | MAP-000 through MAP-006 branches, commits, targets, assets, and tests are present. MAP-007 is implementation-ready if offline DEM import is desired; current runtime PASS is unverified |
| `plans/two_drone_mountain_scenario_plan.md` | Add scripted target plus ArduPilot chaser after MAP-006 | SCN-011 behavior-parity fix is now required for moving-target dual mode | SCN-000 through SCN-010 branches/commits and implementation files are present. SCN-010 proves routing/identity only; it launches no target route controller and therefore does not preserve target motion |

All three plans are tracked in `plans/`. The root-level duplicate names mentioned in the review request are absent, which is correct; `plans/two_drone_mountain_scenario_plan.md` is the canonical two-drone path. The historical `cleaup_plan.md` is a completed DFD-012 replacement note and should remain preserved, not treated as an active plan.

## 7. Mountain terrain map architecture

```text
models/scenarios/mountain_map/scenario.yaml
  -> deterministic PCG64 generator + quantization
  -> committed heightfield.bin (canonical float32 physics asset)
     + heightfield_*.npy, metadata/checksums, nature.json
     + terrain_preview.png (debug/visual only)
  -> canonical scenario loader + MuJoCo MjVfs
  -> hfield collision, terrain height query, safe startup/reset spawn
  -> transient-local ROS terrain/nature MarkerArrays
  -> RViz2/noVNC
```

For `mountain_map`, the loader removes the base model's flat `visual_ground`, loads `mountain_map/heightfield.bin` through `mujoco.MjVfs`, and uses the canonical terrain query to place the spawn and finite launch pad. The binary stores little-endian float32 samples in MuJoCo's required Y-reversed row order; PNG is not a physics input. RViz receives a decimated static terrain triangle mesh plus deterministic tree/rock markers with transient-local durability.

Implemented command surface:

```bash
make generate-map
make test-map
make up-map-viz
```

Key acceptance risks:

- Cross-platform deterministic bytes/checksums and preview-encoder drift.
- Image-row/world-Y inversion, MuJoCo hfield size semantics, and exact VFS asset names.
- Divergence between the software triangle height query and MuJoCo's triangulated collision surface.
- Accidentally retaining `visual_ground`, which can hide hfield contact/ray errors.
- Startup/reset races, pre-step odometry sampling, and `SIM_RESET_AFTER_FRAMES` semantics.
- Marker counts, decimated z bounds, transient-local late delivery, and RViz performance.
- Explicit `SCENARIO=landing_pad` regression isolation and reliable cleanup after every live check.

The commands and committed assets are present, but none of these map commands was run during this review.

## 8. Two-drone scenario architecture

The default MVP path contains one scripted kinematic target already in the sky and one ArduPilot-controlled chaser starting on the mountain launch area. The target flies a deterministic straight XY segment at constant terrain-relative altitude and reverses with `route_behavior: ping_pong`. The chaser arms, takes off, intercepts the target's body-frame formation point, and holds the configured side-by-side offset.

The target is collision-filtered/non-contacting for deterministic execution. Active collision avoidance is out of scope, but the E2E must fail if any sampled `actual_separation_m` falls below `safety.min_drone_separation_m` (currently 3 m). It also checks geofence, terrain clearance, target route/AGL error, chaser altitude gain, continuous formation hold, and final completion.

Mission phases:

```text
IDLE -> HEALTH_CHECK -> ARM -> TAKEOFF -> INTERCEPT
     -> FORMATION_HOLD -> COMPLETE
     `-> ABORT on timeout, health, geofence, enforced-clearance,
         or minimum-separation failure
```

Per-drone odometry, TF, distinguishable markers, route/formation markers, and target/chaser front-camera topics feed the saved two-drone RViz configuration. RViz2 through local noVNC remains the only UI. The default remains the scripted-target MVP; optional SCN-010 adds separate target/chaser SITL services, MAVLink identities, JSON channels, and ROS namespaces behind the explicit dual-SITL profile.

## 9. Known risks and open decisions

### P0 blockers

- None found in repository state: the pre-report tree was clean and all canonical plans were present.

### High

- Full regression status is unknown. `SCENARIO=landing_pad make test-e2e`, `make test-map`, and `CAMERA_ENABLED=1 make test-e2e-two-drone` were not run in this review; the later dual-SITL inspection diagnosed target non-motion but was not a complete regression pass.
- A 2026-07-29 dual-SITL run exposed a product-level gap: `dual_sitl.launch.py` launches two MAVLink bridges but no publisher for `/target/drone/arm` or `/target/drone/cmd_vel`. The target's configured route is never commanded, while the SCN-010 probe can pass after a single heartbeat and odometry sample.
- The implementation branch is published and synchronized, but it is 51 commits ahead of a stale initial `origin/main`. The integration base must still be chosen deliberately; do not branch from current `origin/main`.
- The requested “start MAP-000” next step is stale. MAP-000 through MAP-006 and SCN-000 through SCN-010 already exist in linear history; recreating those branches would duplicate or regress work.

### Medium

- Verifier/cleanup contract drift: `scripts/run_dual_sitl_check.sh` receives a PASS from `dual_sitl_probe.py` and prints the same PASS again; `scripts/run_map004_spawn_check.sh` emits both inner and wrapper PASS lines; `Makefile:test-e2e`, map wrappers, and observability targets can print output after PASS or suppress cleanup failures. The “exactly one final PASS after successful cleanup” contract is not uniformly satisfied.
- Plan metadata drift: the base plan uses `DCK-*`, while history, scripts, and later plans use `DFD-*`; the SCN plan requires nonexistent `DFD-018`; the base-plan extension uses obsolete `ENABLE_CAMERAS` once; the base plan also skips the separate MAP layer in its extension summary.
- Documentation routing is stale: `README.md` calls the already implemented two-drone scenario “planned,” omits the SCN-010 command surface, and `AGENTS.md` still routes work to an exhausted SCN plan.
- `SIM_RESET_AFTER_FRAMES` is implemented but missing from `.env.example` and the README configuration table.
- Mandatory Make targets/tests were found, but `make test` is only a scaffold check and could be mistaken for the full suite. Optional MAP-007 files/targets are absent as expected.
- Early base branch refs `dfd-001` through `dfd-004` collapse onto one commit, so their historical per-ticket boundaries are not independently evidenced.
- Cleanup traps exist, but the regression workflows must prove teardown on success, failure, and interruption; static inspection alone cannot establish this.

### Low

- `README.md` is titled `dstrike`, while the repository/Compose project is `dsriker`.
- The active plans contain minor editorial duplication, including a repeated SCN-009 heading and repeated MAP task label.
- The misspelled historical filename `cleaup_plan.md` is harmless and should be preserved unless a separate documentation ticket supersedes it without deleting history.

## 10. Recommended next actions

The requested baseline sequence remains the correct historical gate, but it has already occurred in this checkout:

1. Clean or commit the working tree coherently; after this review, commit the new report before starting any implementation ticket.
2. Ensure the final plans are committed at `plans/ardupilot_mujoco_ros2_full_docker_plan.md`, `plans/mountain_terrain_map_plan.md`, and `plans/two_drone_mountain_scenario_plan.md`; this is already true at HEAD.
3. Historically, start MAP-000 from the committed MAP-ready/base-stack tip, not stale `main`; do not repeat this now because MAP-000 already exists at `61b614e`.
4. Run or confirm the base regression before MAP work: `SCENARIO=landing_pad make test-e2e`. For the current checkout, rerun it as a regression rather than a MAP gate.
5. Historically, implement MAP tickets in order through MAP-006; this history is present.
6. Only after MAP-006 passes, start SCN-000; that ancestry is present, and the repository has since advanced through SCN-010.

Current next-step sequence:

1. Review and commit this report on a deliberate documentation branch or as an approved documentation commit; leave no unrelated changes.
2. Decide how the published 51-commit implementation branch will be integrated and record the authoritative branch base. Do not use stale `origin/main` as the base.
3. From the current committed tip, run `SCENARIO=landing_pad make test-e2e`, `make test-map`, `CAMERA_ENABLED=1 make test-e2e-two-drone`, and `make test-dual-sitl`, followed by `make down`. Fix failures before claiming the stack is current.
4. Create a narrow, ticketed branch to reconcile verifier cleanup/PASS contracts and stale plan/README/AGENTS metadata if those contracts remain authoritative.
5. Choose the next feature explicitly: optional MAP-007 from the verified current descendant, or a newly written post-SCN plan. Do not recreate MAP-000 or SCN-000.

## 11. Commands for human review

| Command | Review status |
| --- | --- |
| `git status --short` | Run before report creation: no output. Run after report creation: showed only `?? plans/project_status_architecture_review.md`. |
| `git branch --show-current` | Run: `feature/SCN-010-dual-sitl` |
| `git log --oneline --decorate -n 30` | Run successfully; shows MAP-000 through SCN-010 lineage and HEAD `6cd1f87` |
| `make help` | Run successfully; required base/map/SCN targets are listed |
| `docker compose config --quiet` | Run successfully; Compose configuration parses |
| `docker compose --profile baseline --profile mujoco --profile viz --profile dual-sitl ps --all` | Run successfully; no services were listed |
| `SCENARIO=landing_pad make test-e2e` | Not run |
| `make generate-map` | Not run |
| `make test-map` | Not run |
| `make up-map-viz` | Not run; browser signoff not performed |
| `CAMERA_ENABLED=1 make test-e2e-two-drone` | Not run |
| `CAMERA_ENABLED=1 make up-two-drone-viz` | Not run; browser signoff not performed |
| `make test-dual-sitl` | Not run |
| `make down` | Not run; the read-only Compose query already showed no services |

## 12. Final recommendation

The repository should **not** start MAP-000: that ticket and MAP-001 through MAP-006 are already implemented in history, followed by SCN-000 through SCN-010. The repo is structurally beyond the requested start point, but SCN-010 currently provides routing/identity rather than moving-target behavior parity. It is not ready for an authoritative “current stack passes” claim until this report is committed, the mainline integration strategy is resolved, SCN-011 restores target motion, and the Docker regressions above pass.

The safest next implementation branch is `feature/SCN-011-dual-sitl-target-route`, based on the committed report descendant of SCN-010 rather than `origin/main`. It should add a safe airborne handoff, namespaced ArduPilot target route control, and motion-based live acceptance while preserving the default scripted-target path.
