# Mountain Terrain Map Plan

Goal: add one deterministic local mountain/nature terrain map to the existing Dockerized ArduPilot + MuJoCo + ROS 2 + RViz2/noVNC stack.

Plan-specific outcome: `make generate-map`, `make test-map`, and `make up-map-viz` should prove that the same mountain terrain is loaded into MuJoCo for physics and displayed in RViz2 through noVNC, with no cloud services and no host robotics installs.

## Status and scope

This is a small standalone `MAP-*` plan. It starts after the base Docker/RViz stack is already working through the existing `DFD-*` plan.

In scope:

```text
procedural mountain heightfield
local deterministic nature props
MuJoCo hfield physics terrain
safe spawn over terrain
RViz2 terrain/nature markers through noVNC
bounded Docker-only checks
```

Out of scope:

```text
two drones
chase/intercept behavior
per-drone cameras
real-world DEM downloads
ArduPilot multi-vehicle setup
```

## Repository gate before MAP-000

Do not start implementation until the checkout has no unrelated changes and the completed base stack is present.

Required gate:

```bash
git status --short
# expected: no unrelated modified/deleted files
# allowed before MAP-000: the canonical untracked plans/mountain_terrain_map_plan.md draft itself
# allowed in MAP-000 only: AGENTS.md reroute to this plan, README link, Makefile placeholders, verifier script
```

Before saving, reconcile any editor buffer with the canonical repository file:

```text
plans/mountain_terrain_map_plan.md
```

Do not implement from a duplicate unsaved buffer such as `mountain_terrain_map_plan(1).md` or `mountain_terrain_map_plan(2).md`.

The active base Docker/RViz plan must remain in the repository. Do not delete:

```text
plans/ardupilot_mujoco_ros2_full_docker_plan.md
```

Branching rule:

```text
Do not assume `main` is the correct base.
Start MAP-000 from the latest committed tip of the completed DFD stack.
Each later MAP branch starts from the committed or merged predecessor MAP branch.
```

Example only, to be run from the current committed DFD tip, not from stale `main`:

```bash
git log --oneline -1
# confirm this is the latest completed DFD stack commit, for example the DFD-016 tip

git checkout -b feature/MAP-000-map-plan
```

If `origin/main` does not already contain the completed Docker/RViz stack, do not branch from `origin/main` for this plan.

Host `git` and `make` are allowed. ROS 2, MuJoCo, ArduPilot, and robotics Python commands must run through Docker.

## Existing repo contract

- Keep existing base tickets as `DFD-*`. This plan uses only `MAP-*`.
- Docker Desktop on macOS is the only host prerequisite.
- Use `/workspace` inside containers.
- Keep the existing `scenarios.py` loader as the single scenario entry point.
- Current scenario XML resolution is `models/scenarios/<scenario>.xml`; extend this, do not bypass it.
- Do not add `/clock` in this map milestone.
- Preserve `/sim/scenario_markers`.
- Preserve `/sim/drone_markers`.
- Keep ArduPilot JSON boundary behavior from the base plan: NED at the JSON boundary, z-up in MuJoCo/RViz, quaternion-only attitude.
- Camera env var remains `CAMERA_ENABLED`; this map milestone should not introduce `ENABLE_CAMERAS`.
- Every new Make target must be listed in `.PHONY` and `make help`.
- Placeholder help text must be replaced when each target becomes real.
- Every automated verifier prints exactly one success line only after all assertions pass:

```text
PASS: MAP-XXX ...
```

## Researched design constraints

Primary references:

```text
MuJoCo Python VFS/assets docs:
https://mujoco.readthedocs.io/en/latest/python.html#assets

MuJoCo hfield XML docs:
https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-hfield

MuJoCo hfield loader source:
https://github.com/google-deepmind/mujoco/blob/main/src/user/user_objects.cc
```

Design decisions:

- Use a MuJoCo heightfield (`hfield`) for physical terrain.
- Use MuJoCo's custom float32 hfield binary as the canonical physics asset.
- Keep PNG only as a visual/debug preview; do not use PNG precision as the physics contract.
- Compile the XML with `mujoco.MjVfs`; do not use the deprecated `assets` dictionary API.
- The XML `size` tuple means `(radius_x, radius_y, elevation_z, base_z)`, so a `size_m: 300` map uses `radius_x = radius_y = 150`.
- MuJoCo normalizes hfield samples to `[0, 1]`; force exact `0.0` and `1.0` samples in the canonical normalized array so compiled height scale is stable.
- `base_depth_m` is collision thickness below zero, not a vertical offset.
- RViz visualization should use `visualization_msgs/msg/MarkerArray`. Terrain is published once as a static/transient-local decimated `TRIANGLE_LIST`; trees/rocks are simple static/transient-local markers.
- A bilinear software `height_at()` is not automatically identical to MuJoCo's triangulated collision surface. Add MuJoCo hfield-data, terrain-filtered ray, real-contact, and live-spawn comparisons before trusting spawn placement.

## Canonical terrain coordinate contract

Define this contract once and test it. Do not let the generator, height query, MuJoCo XML, and RViz markers each invent their own mapping.

### Map extent

```text
size_m             total square map width in meters
radius_m           size_m / 2
world x range      [-radius_m, +radius_m]
world y range      [-radius_m, +radius_m]
geom origin        hfield geom at world position (0, 0, 0)
```

For `size_m: 300`:

```text
x, y in [-150, +150]
MuJoCo hfield size = "150 150 <max_height_m> <base_depth_m>"
```

### Canonical image-style row/column to world mapping

Use a square `resolution x resolution` grid with odd `resolution >= 65`.

The canonical generated arrays are image-style:

```text
col 0              x = -radius_m
col resolution-1   x = +radius_m
row 0              y = +radius_m
row resolution-1   y = -radius_m
```

Formula:

```text
x = -radius_m + col * size_m / (resolution - 1)
y = +radius_m - row * size_m / (resolution - 1)
```

The inverse mapping must be tested at corners, center, spawn, and several interior points.

### MuJoCo custom-binary row order

MuJoCo's PNG hfield loader reverses image rows, but the custom binary hfield loader copies rows directly. A custom binary row 0 maps to negative Y in the compiled model. Therefore:

```text
height_norm[row, col]        canonical image-style array; row 0 = +Y
heightfield.bin data         height_norm[::-1, :] written row-major; binary row 0 = -Y
compiled model.hfield_data   expected to match height_norm[::-1, :] after MuJoCo normalization
RViz markers                 use canonical image-style height_norm, not the reversed binary array
software height_at()         uses canonical image-style height_norm
```

Never compare `model.hfield_data` to the whole flat shared buffer directly. Slice by hfield ID/address:

```python
hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "mountain_hfield")
adr = model.hfield_adr[hfield_id]
nrow = model.hfield_nrow[hfield_id]
ncol = model.hfield_ncol[hfield_id]
compiled = model.hfield_data[adr:adr + nrow * ncol].reshape(nrow, ncol)
expected = height_norm[::-1, :]
```

The verifier must include asymmetric corner samples and at least three asymmetric interior samples so a Y inversion cannot pass accidentally.

### Canonical physics height asset

Use a normalized float32 array as the canonical source for MuJoCo physics:

```text
height_norm[row, col] = float32 in [0.0, 1.0]
height_m[row, col]    = height_norm[row, col] * max_height_m
```

Generated files:

```text
heightfield.bin          canonical MuJoCo custom hfield binary, used by physics; stores height_norm[::-1, :]
heightfield_norm.npy     exact canonical image-style normalized float32 array for tests
heightfield_m.npy        exact canonical image-style meter-valued float32 array for tests/debugging
terrain_preview.png      visual preview only; not used for physics precision
terrain_meta.json        config, dimensions, sentinel points, and checksums
nature.json              deterministic trees/rocks
```

`heightfield.bin` format:

```text
(int32 little-endian)   nrow
(int32 little-endian)   ncol
(float32 little-endian) data[nrow*ncol], row-major, containing height_norm[::-1, :]
```

Normalization and determinism requirements:

```text
min(height_norm) == 0.0 exactly
max(height_norm) == 1.0 exactly
all values are finite
all values are in [0.0, 1.0]
corner samples are asymmetric enough to detect row/Y inversion
use a specified PRNG algorithm, e.g. numpy.random.PCG64 with explicit seed
quantize generated continuous values to a documented grid before casting to float32
```

Use deterministic quantization before `float32` conversion so fixtures remain byte-identical across supported Mac architectures and container platforms. A recommended rule is:

```text
height_norm_quantized = round(height_norm_float64 * 1_000_000) / 1_000_000
height_norm = height_norm_quantized.astype("<f4")
```

Do not assume PNG encodes physics height. The PNG preview may be 8-bit or 16-bit, but it is a review artifact only.

### Checksums and committed fixtures

Generated assets are committed fixtures. Regeneration must be deterministic.

Store in `terrain_meta.json`:

```text
config_sha256             canonical JSON form of scenario config
height_norm_sha256        raw little-endian float32 canonical image-style height_norm bytes
height_m_sha256           raw little-endian float32 canonical image-style height_m bytes
hfield_binary_sha256      encoded MuJoCo hfield binary bytes containing height_norm[::-1, :]
preview_png_sha256        encoded preview PNG bytes
nature_sha256             canonical JSON of nature props
manifest_sha256           aggregate of the above fields
sentinel_points           exact row/col/x/y/height_norm/height_m for corners, min, max, spawn, peak, valley
```

Do not use PNG bytes as the only deterministic signal; PNG encoder metadata can drift.

### Terrain interpolation contract

Use one exact software triangle interpolator for terrain height checks, not an informal bilinear helper.

For a grid cell with corners:

```text
(row, col)       upper-left in canonical image-style coordinates
(row, col + 1)   upper-right
(row + 1, col)   lower-left
(row + 1, col+1) lower-right
```

Define the diagonal used for terrain triangle interpolation and use it consistently in:

```text
src/physics/terrain_map.py
tests/test_terrain_map.py
scripts/mujoco_terrain_collision_probe.py
scripts/map_marker_probe.py
```

The collision probe must include two points inside the same deliberately non-coplanar cell, one on each side of the chosen diagonal, so the test proves the triangle interpolator agrees with MuJoCo's actual triangulation.

### Marker count and decimated bounds contract

For the default config:

```text
resolution = 129
rviz_decimation = 4
boundary-inclusive samples per side = ((129 - 1) / 4) + 1 = 33
terrain cells = 32 * 32 = 1024
terrain triangles = 2048
TRIANGLE_LIST points = 6144
```

Marker probes must compute this formula from config and assert the exact result. For the default config, the expected terrain point count is exactly `6144`.

Z-bounds for RViz marker verification must be computed from the decimated canonical array actually published to RViz, not from the global terrain min/max. The global min/max may not fall on a decimation-aligned sample.

## Revised Make target ownership

```text
MAP-000   placeholder targets only; placeholders are expected to fail clearly
MAP-002   implement generate-map
MAP-005   implement up-map-viz
MAP-006   implement test-map
```

No ticket may call a Make target before the ticket that owns it has implemented that target.

---

## MAP-000 — Add map plan, README link, and tested placeholders

**Codex task**

Add this standalone plan to the repository and link it from the README without claiming the feature works yet.

Files:

```text
AGENTS.md                     # only if the local workflow intentionally points Codex to this plan
plans/mountain_terrain_map_plan.md
README.md
Makefile
scripts/verify_map000_contract.py
```

Requirements:

- Start from the latest committed completed DFD stack tip, not from stale `main` unless `main` already contains that tip.
- Add a README section named `Mountain terrain map`.
- Keep the active base Docker plan in the repo.
- Add Make placeholders that fail clearly until implemented:
  - `generate-map` says it is implemented by `MAP-002`.
  - `up-map-viz` says it is implemented by `MAP-005`.
  - `test-map` says it is implemented by `MAP-006`.
- Add all three placeholders to `.PHONY` and `make help`.
- The placeholders must exit nonzero and print the expected message.
- Do not use `make ... || true` as acceptance. The verifier must run each placeholder, assert nonzero exit, and assert the expected text.
- Add `scripts/verify_map000_contract.py` that confirms:
  - placeholders fail for the expected reason;
  - `.PHONY` and `make help` include the new targets;
  - the README link exists;
  - no obsolete non-DFD Docker ticket prefix drift appears in this plan;
  - no branch instructions tell Codex to branch from stale `main` unconditionally;
  - the plan allows the untracked map plan draft itself while rejecting unrelated changes;
  - the active base plan has not been deleted.

**Definition of Done**

```bash
docker compose run --rm physics bash -lc "cd /workspace && python3 scripts/verify_map000_contract.py"
```

Done when the verifier prints:

```text
PASS: MAP-000 map plan and intentionally failing placeholders are correct
```

---

## MAP-001 — Extend canonical scenario loading and add container dependencies

**Codex task**

Extend the existing `scenarios.py` API so `SCENARIO=mountain_map` is still loaded through the canonical scenario loader.

Files:

```text
src/physics/scenarios.py
src/physics/scenario_config.py
models/scenarios/mountain_map.xml
models/scenarios/mountain_map/scenario.yaml
docker/Dockerfile
docker/requirements.txt or equivalent dependency file
scripts/verify_map001_scenario_loader.py
```

Scenario layout:

```text
models/scenarios/mountain_map.xml                  # canonical XML entry path
models/scenarios/mountain_map/scenario.yaml        # map config
models/scenarios/mountain_map/heightfield.bin      # created in MAP-002
models/scenarios/mountain_map/heightfield_norm.npy # created in MAP-002
models/scenarios/mountain_map/heightfield_m.npy    # created in MAP-002
models/scenarios/mountain_map/terrain_preview.png  # created in MAP-002
models/scenarios/mountain_map/nature.json          # created in MAP-002
models/scenarios/mountain_map/terrain_meta.json    # created in MAP-002
```

Minimum config:

```yaml
schema_version: 1
name: mountain_map
frame_id: world
replaces_ground_plane: true
terrain:
  mode: procedural
  prng: numpy.random.PCG64
  quantization_decimals: 6
  seed: 7
  size_m: 300
  resolution: 129
  max_height_m: 45
  base_depth_m: 3
  spawn_clearance_m: 1.5
  rviz_decimation: 4
  spawn_basin_radius_m: 20
  spawn_basin_max_slope_deg: 5
  spawn_basin_max_relief_m: 1.0
nature:
  seed: 11
  trees: 80
  rocks: 35
  tree_min_radius_m: 35
  avoid_spawn_radius_m: 20
  max_slope_deg: 28
spawn:
  x_m: -90
  y_m: -40
  heading_deg: 0
```

Loader requirements:

- Keep `models/scenarios/<name>.xml` as the primary scenario lookup.
- Return a `ScenarioBundle` or equivalent with:
  - `name`
  - `xml_string`
  - `vfs_files: dict[str, bytes]` or equivalent internal byte map used only to populate `mujoco.MjVfs`
  - `config`
- Default and existing scenarios return empty `vfs_files` and keep current behavior.
- `mountain_map` validates `scenario.yaml` but does not require generated assets until `MAP-003`.
- Reject unknown config keys.
- Reject NaN/Inf and non-finite numeric values.
- Validate `schema_version == 1`.
- Validate `name == mountain_map`.
- Validate `frame_id == world`.
- Validate `terrain.mode == procedural`.
- Validate `terrain.prng == numpy.random.PCG64`.
- Validate `terrain.quantization_decimals` is a positive integer.
- Validate integer `seed`, `trees`, `rocks`, and `resolution` fields.
- Validate nonnegative `trees` and `rocks`.
- Validate odd `resolution >= 65`.
- Validate positive integer `rviz_decimation` and `(resolution - 1) % rviz_decimation == 0`.
- Validate positive `size_m`, `max_height_m`, `base_depth_m`, `spawn_clearance_m`, `spawn_basin_radius_m`, `tree_min_radius_m`, and `avoid_spawn_radius_m`.
- Validate `spawn_basin_max_slope_deg`, `spawn_basin_max_relief_m`, and `nature.max_slope_deg` are finite and nonnegative.
- Validate spawn point is inside map bounds.
- Validate the full spawn basin is inside map bounds:
  - `abs(spawn.x_m) + spawn_basin_radius_m <= size_m / 2`
  - `abs(spawn.y_m) + spawn_basin_radius_m <= size_m / 2`
- Define `tree_min_radius_m` precisely: every tree trunk center must satisfy `sqrt(x^2 + y^2) >= tree_min_radius_m`, allowing a clear center-region view in RViz.
- Validate `replaces_ground_plane: true` is present for this map.
- Define heading conversion once:
  - `heading_deg` is yaw about z-up `+Z`.
  - `0° = +X`.
  - positive heading is counter-clockwise around `+Z`.
  - heading is normalized modulo `360`.
  - z-up quaternion is `wxyz = [cos(yaw/2), 0, 0, sin(yaw/2)]`.
- Tests must cover heading `0°`, `+90°`, and `-90°`, not only the configured zero case.
- Add pinned dependencies for YAML, PNG preview generation, and deterministic arrays, for example PyYAML, Pillow, and NumPy, in the Docker image.
- Pin the MuJoCo Python package version instead of leaving it floating.
- Rebuild the image and verify imports inside Docker.

**Definition of Done**

```bash
make build
docker compose run --rm physics bash -lc "cd /workspace && python3 scripts/verify_map001_scenario_loader.py"
```

Done when the verifier confirms loader behavior, config schema, Docker imports, pinned dependency versions, heading conversion, and default-scenario compatibility, then prints:

```text
PASS: MAP-001 mountain_map is available through the canonical scenario loader
```

---

## MAP-002 — Generate deterministic terrain and nature assets

**Codex task**

Implement local deterministic map generation and own the `generate-map` target.

Files:

```text
Makefile
scripts/generate_terrain_map.py
scripts/verify_map002_generation.py
src/physics/terrain_generator.py
src/physics/nature_generator.py
models/scenarios/mountain_map/heightfield.bin
models/scenarios/mountain_map/heightfield_norm.npy
models/scenarios/mountain_map/heightfield_m.npy
models/scenarios/mountain_map/terrain_preview.png
models/scenarios/mountain_map/nature.json
models/scenarios/mountain_map/terrain_meta.json
```

Generation requirements:

- Implement `make generate-map` in this ticket.
- Add `generate-map` to `.PHONY` and replace its `make help` placeholder with the real command description.
- `make generate-map` must invoke Python through Docker only, for example:

```make
	docker compose run --rm physics bash -lc "cd /workspace && python3 scripts/generate_terrain_map.py --scenario mountain_map && python3 scripts/verify_map002_generation.py --scenario mountain_map"
```

- Do not run host Python for generation or verification.
- Use only local deterministic generation; no network access.
- Use `numpy.random.PCG64` with explicit seeds from config.
- Quantize generated continuous values using the configured rule before casting to little-endian float32.
- Generate a mountain-like heightfield with:
  - several broad peaks;
  - lower-frequency noise;
  - a quantitative spawn basin around configured spawn;
  - no NaN/Inf values;
  - normalized heights in `[0, 1]`;
  - meter heights in `[0, max_height_m]`.
- The spawn basin must satisfy all configured limits, not just look flatter:
  - all sampled points within `spawn_basin_radius_m` have slope `<= spawn_basin_max_slope_deg`;
  - relief inside the basin is `<= spawn_basin_max_relief_m`;
  - the configured spawn point is inside the basin.
- Force exact `0.0` and `1.0` samples in `height_norm` away from the spawn basin.
- Ensure the four corner samples are asymmetric so tests can detect row/Y inversion.
- Write `heightfield.bin` in MuJoCo custom hfield format:
  - `int32 nrow`, `int32 ncol`, then row-major `float32 height_norm[::-1, :]`.
- Write `heightfield_norm.npy` and `heightfield_m.npy` in canonical image-style row order.
- Generate `terrain_preview.png` for manual review only; do not use it for physics precision.
- Generate deterministic `nature.json` containing trees and rocks with:
  - stable IDs;
  - x/y/z positions;
  - yaw/scale;
  - terrain-aligned z;
  - map-bound checks;
  - spawn-exclusion checks;
  - `tree_min_radius_m` checks;
  - slope-limit checks.
- Commit generated fixtures.
- Determinism verifier must:
  - generate twice into two temporary directories and compare bytes;
  - compare generated temp artifacts against the committed fixtures;
  - include untracked file detection under `models/scenarios/mountain_map`;
  - verify all checksums in `terrain_meta.json`;
  - verify `heightfield.bin` contains exactly `height_norm[::-1, :]`, not canonical row order.
- `make generate-map` must fail if the scenario config is invalid and must print `PASS: MAP-002 ...` only after deterministic checks pass.

**Definition of Done**

```bash
make generate-map
```

Done when generation succeeds, deterministic temp-dir checks pass, expected files exist, committed fixtures match regenerated bytes, there are no untracked generated assets, and the command prints:

```text
PASS: MAP-002 deterministic mountain terrain and nature assets generated
```

---

## MAP-003 — Load MuJoCo heightfield through MjVfs and verify collision

**Codex task**

Load the generated terrain into MuJoCo as an `hfield` using the existing scenario loader and an explicit `mujoco.MjVfs`.

Files:

```text
src/physics/scenarios.py
src/physics/mujoco_quad.py
src/physics/mujoco_model_builder.py or existing XML merge utility
scripts/mujoco_load_scenario.py
scripts/mujoco_terrain_collision_probe.py
models/scenarios/mountain_map.xml
```

XML/model requirements:

- `mountain_map.xml` is a template whose hfield size is patched from `scenario.yaml` before compilation. Do not hard-code `150 150 45 3` as the source of truth.
- A valid post-patch hfield entry for the default config is equivalent to:

```xml
<asset>
  <hfield name="mountain_hfield" file="mountain_map/heightfield.bin" size="150 150 45 3"/>
</asset>
<worldbody>
  <geom name="mountain_terrain" type="hfield" hfield="mountain_hfield" pos="0 0 0"/>
</worldbody>
```

- Compute hfield `size` from config:

```text
radius_x = size_m / 2
radius_y = size_m / 2
elevation_z = max_height_m
base_z = base_depth_m
```

- Populate `mujoco.MjVfs` with `heightfield.bin` bytes under the exact filename referenced by XML.
- Compile with `vfs`, not the deprecated asset dictionary.
- Do not rely on the process working directory to find `heightfield.bin`.
- If assets are missing, fail with a clear message:

```text
mountain_map assets missing; run `make generate-map`
```

- Do not bypass `scenarios.py`.
- Preserve default flat-ground scenarios.

Ground-plane policy:

- For `mountain_map`, remove the base `visual_ground` plane before model compilation.
- Do not merely set `contype=0` or `conaffinity=0`; disabled planes can still confuse ray queries.
- Preserve the base flat plane for default and existing scenarios.
- A parsing-only test is not enough; verify the physical collision terrain is active.

Collision verifier requirements:

`scripts/mujoco_terrain_collision_probe.py` must verify:

- `mountain_terrain` geom exists.
- `visual_ground` geom does not exist in the compiled `mountain_map` model.
- hfield size matches the coordinate contract exactly.
- compiled `model.hfield_data` is sliced by hfield ID/address, then reshaped to `(nrow, ncol)`.
- the compiled hfield slice matches `heightfield_norm.npy[::-1, :]` at:
  - exact min and max sentinel samples;
  - four asymmetric corners;
  - center;
  - configured spawn;
  - detected peak;
  - detected valley;
  - at least three deterministic interior points.
- no row/Y inversion occurs.
- default scenario still has its expected flat ground behavior.
- terrain-filtered vertical ray comparisons succeed at:
  - four points just inside the valid boundary;
  - center;
  - configured spawn point;
  - detected peak;
  - detected valley;
  - at least three deterministic interior points;
  - two points in a deliberately non-coplanar cell, one on each side of the chosen terrain diagonal.
- Every ray assertion must:
  - exclude the drone body/base link geom from the query;
  - assert the returned geom is exactly `mountain_terrain`;
  - compare against the exact software triangle interpolator.
- A separate real contact test must create/drop a small collision-enabled probe object above the terrain and assert:
  - `data.ncon > 0` after settling;
  - at least one contact pair contains the `mountain_terrain` geom ID;
  - the probe rests at the expected terrain height within the mandatory tolerance.
- Differences between expected triangulated terrain height and MuJoCo ray/contact height must be `<= 0.10 m` unless the verifier documents and enforces a tighter bound.

**Definition of Done**

```bash
docker compose run --rm physics bash -lc "cd /workspace && SCENARIO=mountain_map python3 scripts/mujoco_terrain_collision_probe.py"
docker compose run --rm physics bash -lc "cd /workspace && SCENARIO=default python3 scripts/mujoco_load_scenario.py --scenario default"
```

Done when both commands pass and the mountain collision probe prints:

```text
PASS: MAP-003 MuJoCo VFS heightfield, row order, ground-plane policy, and terrain collision are verified
```

---

## MAP-004 — Apply terrain-aware spawn on startup and every reset

**Codex task**

Wire the terrain-aware spawn into the live simulator instead of only calculating it offline.

Files:

```text
compose.yaml or docker-compose.yml
src/physics/terrain_map.py
src/physics/mujoco_quad.py
src/physics/mujoco_json.py
scripts/terrain_probe.py
scripts/run_map004_spawn_check.sh
scripts/ros_ground_truth_spawn_probe.py
tests/test_terrain_map.py
```

Requirements:

- Implement `height_at(x_m, y_m)` using the canonical image-style float32 terrain array and coordinate contract.
- Implement `physics_height_at(x_m, y_m)` or equivalent MuJoCo ray/contact-backed check for parity tests.
- Implement `safe_spawn_pose(config)`:

```text
x = config.spawn.x_m
y = config.spawn.y_m
z = terrain_collision_height(x, y) + spawn_clearance_m
heading = config.spawn.heading_deg
quat_wxyz = z-up yaw quaternion from MAP-001
```

- Raise a clear error for out-of-bounds x/y; do not silently clamp in v1.
- Use terrain-aware spawn on initial simulator startup.
- Use the same terrain-aware spawn on every reset path, including frame-count resets in `mujoco_json.py`.
- Add a deterministic reset trigger for tests:

```text
SIM_RESET_AFTER_FRAMES=<N>
```

- Add the reset variable to Compose so it actually reaches the container:

```yaml
SIM_RESET_AFTER_FRAMES: ${SIM_RESET_AFTER_FRAMES:-0}
```

- Define `SIM_RESET_AFTER_FRAMES=0` as disabled.
- Reject non-integer or negative values.
- Positive values mean reset exactly every `N` accepted PWM packets, not every wall-clock frame and not every loop iteration without an accepted packet.
- The simulator must publish one `/sim/ground_truth/odom` sample immediately after applying startup/reset spawn and before the next `mj_step`, so the pre-fall pose is observable.
- Verify live `/sim/ground_truth/odom`, not just local calculations.
- Do not use absolute altitude as success. Use terrain clearance or altitude gain from the configured spawn height.

Subscriber-first odometry choreography:

- The startup/reset probe must avoid volatile-QoS races.
- Start ROS 2 services required for discovery first.
- Launch `ros_ground_truth_spawn_probe.py` before starting physics and SITL.
- The probe must print a `READY` line only after its `/sim/ground_truth/odom` subscription is active.
- Only after `READY`, start the physics service and SITL.
- Keep the same subscriber process alive through startup and both reset checks.
- The probe must identify and assert three pre-step samples:
  - startup spawn sample;
  - first reset spawn sample;
  - second reset spawn sample.

Unit test requirements:

- `tests/test_terrain_map.py` must test coordinate conversion, out-of-bounds handling, exact triangle interpolation, heading quaternion conversion, and terrain/MuJoCo parity at sentinel points.
- Test headings `0°`, `+90°`, `-90°`, and an equivalent modulo-360 heading.
- Mandatory numeric tolerances:

```text
x/y spawn tolerance <= 0.25 m
z spawn tolerance <= 0.25 m
heading tolerance <= 1 degree
terrain collision tolerance <= 0.10 m
```

Automated verifier requirements:

`scripts/run_map004_spawn_check.sh` must:

- install a shell trap for cleanup on success, failure, and interruption;
- call `make down` before starting;
- run `tests/test_terrain_map.py` inside Docker;
- use subscriber-first choreography;
- start the stack with `SCENARIO=mountain_map` using Docker Compose;
- set `SIM_RESET_AFTER_FRAMES` to a positive value that produces at least two resets within the bounded test window;
- wait with bounded timeouts;
- verify live ground-truth x/y is near configured spawn;
- verify live ground-truth z equals terrain height plus clearance within tolerance;
- verify the first observed odom sample after startup/reset is the pre-step spawn sample;
- observe at least two simulator resets and verify the pose returns to the same safe spawn;
- call `make down` at the end.

**Definition of Done**

```bash
SCENARIO=mountain_map bash scripts/run_map004_spawn_check.sh
```

Done when terrain unit tests, live startup pose, and live reset pose checks pass, and the script prints:

```text
PASS: MAP-004 terrain-aware startup and reset spawn are verified
```

---

## MAP-005 — Publish terrain/nature markers and implement map visualization target

**Codex task**

Publish a local RViz2 view of the mountain/nature map and own the `up-map-viz` target.

Files:

```text
Makefile
compose.yaml or docker-compose.yml
src/physics/ros_visualization.py
scripts/start_rviz_novnc.sh
scripts/map_marker_probe.py
scripts/run_map005_marker_check.sh
config/mountain_map.rviz
config/drone.rviz
README.md
```

ROS topics:

```text
/sim/terrain_markers
/sim/nature_markers
/sim/scenario_markers    # preserve existing topic
/sim/drone_markers       # preserve existing topic
/sim/ground_truth/odom
```

Requirements:

- Implement `make up-map-viz` in this ticket.
- Add `up-map-viz` to `.PHONY` and replace its `make help` placeholder with the real command description.
- `make up-map-viz` should:
  - run `make generate-map` or fail with the same clear missing-assets message;
  - set `SCENARIO=mountain_map`;
  - pass the selected RViz config into the container through `compose.yaml` and `scripts/start_rviz_novnc.sh`;
  - use `config/mountain_map.rviz` or an explicit `RVIZ_CONFIG` variable;
  - expose noVNC at `127.0.0.1:6080`;
  - be local-only.
- Publish terrain once as a static/transient-local `MarkerArray` with nonempty `TRIANGLE_LIST` data. Do not republish thousands of points at 10 Hz.
- Publish trees/rocks from `nature.json` once as deterministic static/transient-local visual markers.
- Use frame `world`.
- Keep `/sim/scenario_markers` published and visible.
- Keep `/sim/drone_markers` published and visible.
- Add a map-specific RViz config centered and scaled for the 300 m map. The existing origin-centered 8 m view is not sufficient.

Marker probe requirements:

`scripts/map_marker_probe.py` must use a late-subscriber transient-local check and validate:

- topic exists;
- message type is `visualization_msgs/msg/MarkerArray`;
- marker frame is `world`;
- terrain contains at least one `TRIANGLE_LIST`;
- terrain vertices are finite;
- terrain x/y bounds equal the configured map bounds within mandatory tolerances;
- terrain z-bounds equal the min/max of the decimated canonical array, not global min/max;
- terrain triangle/point count matches the configured decimation exactly;
- for default config, terrain has exactly `2048` triangles and `6144` `TRIANGLE_LIST` points;
- nature tree/rock counts match `nature.json` exactly;
- default config has exactly `80` trees and `35` rocks;
- spawn exclusion zone contains no tree/rock markers;
- `/sim/scenario_markers` is still available;
- `/sim/drone_markers` is still available;
- late-subscriber delivery works after the publisher has already started.

Mandatory marker tolerances:

```text
x/y bound tolerance <= 0.01 m
z bound tolerance <= 0.05 m
nature marker count tolerance = 0
terrain point count tolerance = 0
```

Automated wrapper requirements:

`scripts/run_map005_marker_check.sh` must:

- call `make down` before starting;
- install a cleanup trap;
- start the map visualization stack with bounded startup waits;
- run `scripts/map_marker_probe.py` inside Docker, not with host ROS 2;
- call `make down` after success, failure, or interruption.

Automated check:

```bash
SCENARIO=mountain_map bash scripts/run_map005_marker_check.sh
```

Manual visual signoff:

```bash
make up-map-viz
# open http://localhost:6080/vnc.html?autoconnect=true&resize=scale
# confirm RViz2 shows the terrain surface, trees/rocks, /sim/scenario_markers, and /sim/drone_markers
make down
```

Done when the automated marker probe prints:

```text
PASS: MAP-005 RViz terrain and nature markers are content-verified
```

The manual RViz view is signed off separately in the README checklist.

---

## MAP-006 — Implement bounded map smoke workflow and regressions

**Codex task**

Own `make test-map` and turn the map work into a safe repeatable workflow.

Files:

```text
Makefile
README.md
scripts/map_smoke_test.py
scripts/run_map_smoke.sh
scripts/ros_wait_terrain_clearance.py
```

Requirements:

- Implement `make test-map` in this ticket.
- Add `test-map` to `.PHONY` and replace its `make help` placeholder with the real command description.
- `make test-map` must clean up before and after, including success, failure, and interruption.
- Use bounded readiness checks. No infinite waits.
- Reuse existing helpers where appropriate:
  - `ros_wait_altitude.py` only if checking altitude gain, not absolute altitude;
  - `ros_topic_probe.py` only for non-static volatile topics;
  - `image_probe.py` only if camera checks are explicitly enabled later.
- For static terrain markers, use `map_marker_probe.py`, not the existing generic topic probe.
- Do not add host `ros2` commands.
- Add an EKF/pre-arm health gate if the stack arms/takes off during this test.
- Pin newly added Docker dependencies.
- Document cleanup and rerun steps.

`make test-map` must verify, in bounded Docker-only commands:

```text
base docker compose config is valid
scenario loader works for default and mountain_map
make generate-map is deterministic
MuJoCo loads mountain_map using MjVfs
flat default scenario still works
mountain_map removes flat visual_ground from the compiled model
terrain collision probe passes with ray and real-contact assertions
safe spawn is applied on startup and reset
terrain/nature markers are content-verified
existing default landing-pad E2E regression passes
existing scenario/quad/visualization regression suites pass
```

Mandatory regression commands:

```bash
SCENARIO=landing_pad make test-e2e
docker compose run --rm physics bash -lc "cd /workspace && pytest tests/test_scenarios.py tests/test_mujoco_quad.py tests/test_ros_visualization.py tests/test_terrain_map.py"
```

`SCENARIO=landing_pad` must be set explicitly so this regression cannot inherit `mountain_map` from the caller's shell.

`make test-e2e` may tear down the visualization stack. Run `make up-map-viz` again after `make test-map` for manual RViz signoff.

If a map takeoff check is added, it must assert one of:

```text
terrain clearance >= configured clearance
altitude gain from initial spawn >= configured gain
```

It must not use only `absolute z > 1`, because mountain spawn altitude can be high at startup.

**Definition of Done**

```bash
make test-map
make up-map-viz
# manual noVNC RViz signoff
make down
```

Done when `make test-map` passes, the manual RViz checklist is complete, `make down` leaves no stack running, and the automated workflow prints:

```text
PASS: MAP-006 map generation, physics terrain, safe spawn, markers, regressions, and cleanup are verified
```

---

## MAP-007 — Optional later: offline real DEM import fixture

**Codex task**

Add real DEM import only after the procedural MVP is stable.

Files:

```text
scripts/import_dem_map.py
src/physics/dem_importer.py
models/scenarios/demo_dem.xml
models/scenarios/demo_dem/scenario.yaml
models/scenarios/demo_dem/source/sample_dem.*
models/scenarios/demo_dem/heightfield.bin
models/scenarios/demo_dem/heightfield_norm.npy
models/scenarios/demo_dem/heightfield_m.npy
models/scenarios/demo_dem/terrain_preview.png
models/scenarios/demo_dem/nature.json
models/scenarios/demo_dem/terrain_meta.json
scripts/mujoco_terrain_collision_probe.py
README.md
```

Requirements:

- Keep this separate from the procedural MVP.
- Do not download data automatically.
- Use a committed small offline DEM fixture.
- Explicitly declare and pin any new dependencies, such as raster/GeoTIFF tooling.
- Convert the DEM into the same artifact contract:

```text
heightfield.bin           stores canonical height_norm[::-1, :] for MuJoCo
heightfield_norm.npy      canonical image-style array
heightfield_m.npy         canonical image-style array
terrain_preview.png
nature.json or empty nature.json
terrain_meta.json
```

- Reuse the same coordinate and checksum contract.
- Reuse the same MuJoCo `MjVfs` path.
- Generalize `scripts/mujoco_terrain_collision_probe.py` so it can verify both `mountain_map` and `demo_dem`.

**Definition of Done**

```bash
docker compose run --rm physics bash -lc "cd /workspace && python3 scripts/import_dem_map.py --input models/scenarios/demo_dem/source/sample_dem.tif --scenario demo_dem --offline-only"
docker compose run --rm physics bash -lc "cd /workspace && SCENARIO=demo_dem python3 scripts/mujoco_terrain_collision_probe.py"
```

Done when a committed local DEM fixture converts and loads without network access, and the command prints:

```text
PASS: MAP-007 offline DEM fixture import works without network downloads
```

## Recommended branch order

Every branch starts from the committed or merged predecessor. The first branch starts from the latest committed completed DFD stack tip, not necessarily `main`.

```text
feature/MAP-000-map-plan
feature/MAP-001-scenario-loader-contract
feature/MAP-002-generate-map-assets
feature/MAP-003-mujoco-hfield-collision
feature/MAP-004-terrain-aware-spawn
feature/MAP-005-rviz-map-markers
feature/MAP-006-map-smoke-workflow
feature/MAP-007-offline-dem-fixture   # optional later
```

## Final acceptance for the smaller milestone

Run:

```bash
make generate-map
make test-map
make up-map-viz
# manual noVNC RViz signoff
make down
```

Then manually open while `make up-map-viz` is running:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

The milestone is complete when:

```text
mountain terrain is committed as deterministic local fixtures
MuJoCo loads the heightfield through MjVfs
compiled model.hfield_data slice matches the canonical float32 terrain contract with the expected Y-reversed binary layout
flat ground is removed from mountain_map
ray checks hit mountain_terrain and real contact includes mountain_terrain
spawn and reset start above the terrain surface
RViz2 shows mountain terrain and nature markers
existing /sim/scenario_markers is still visible
existing /sim/drone_markers is still visible
all tests use bounded Docker-only commands
make down reliably cleans the stack
no cloud service or external provider is required
```

## Implementation notes for Codex

- Codex owns narrow implementation details within each ticket. It may choose the simplest repo-native design for helper functions, internal class/API shape, triangle interpolation implementation, DDS/reset correlation, probe geometry, dependency organization, and cleanup mechanics, provided every stated contract, backward-compatibility requirement, bounded verifier, regression, Definition of Done, and exact PASS line succeeds.
- Codex must update the plan before proceeding only when a decision would change a public interface, Make target, ROS topic, environment variable, artifact format, coordinate convention, ticket scope/order, or acceptance behavior.
- Keep the plan and implementation synchronized. Do not change Make target names, topic names, env vars, or PASS line format without updating this plan.
- Prefer small verifier scripts over long README command sequences. Each script should assert its own prerequisites and print exactly one `PASS: MAP-XXX ...` line only after all checks pass.
- Keep automated verification separate from manual RViz visibility signoff.
- If a ticket discovers the current repo differs from this plan, stop and update the plan before implementing the next ticket.
