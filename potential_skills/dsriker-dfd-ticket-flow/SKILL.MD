# Dsriker DFD Ticket Flow

Use this proposed skill when a user asks to implement a numbered DFD ticket in
`/Users/leon/Desktop/git/dsriker`, such as `implement 015`.

## Context to gather

1. Read `AGENTS.md` and the matching DFD section in
   `ardupilot_mujoco_ros2_full_docker_plan.md`.
2. Check `git status --short --branch` and confirm the previous ticket branch is
   clean and committed.
3. Create one branch per ticket from the previous ticket tip.
4. Inspect the nearby Makefile, Compose, README, scripts, tests, `.env.example`,
   and checkout-local `.env` before editing.

## Ticket workflow

1. Implement only the selected DFD scope.
2. Keep the stack Docker Desktop for Mac only; do not require host ROS, Python,
   MuJoCo, ArduPilot, or non-Docker runtimes.
3. Prefer Docker-only verification. Start with `docker compose config` after
   Compose edits, then run the exact Make or Docker commands from the DFD.
4. When live services are required, use the matching Compose profile and clean up
   with `make down` or the equivalent profiled `docker compose down`.
5. Update `README.md` with concrete manual test steps and the real PASS line.

## Recurring pitfalls

- `.git` writes may need an approved unsandboxed git command.
- Docker socket access may need approval; treat that as an environment boundary,
  not a code failure.
- Local `.env` can override `.env.example`, especially ports and topic defaults.
- ArduPilot JSON mode expects numeric container IPv4 targets, not Compose service
  names, in SITL arguments.
- If `pytest` is required literally, the image must expose a `pytest` executable.
