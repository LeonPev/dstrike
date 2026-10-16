---
name: inspect-ardupilot-mavlink-contract
description: Inspect the dsriker project's pinned ArduPilot and pymavlink sources to establish MAVLink field names, units, senders, update rates, and coordinate origins. Use before adding or debugging SITL telemetry conversions when remembered protocol details are uncertain.
---

# Inspect ArduPilot MAVLink Contract

Keep all inspection inside the project Docker image. Do not install ArduPilot,
pymavlink, or protocol tooling on the host.

1. Read `compose.yaml`, `docker/Dockerfile`, and `docker/requirements.txt` to record
   the exact ArduPilot commit, pymavlink version, service, and startup arguments.
2. Inspect `/opt/ardupilot` in the relevant Compose service. Find the message
   sender, simulator backend, and origin conversion with `rg`; do not infer
   coordinate semantics from message names alone.
3. Inspect the installed pymavlink dialect definition for the exact message
   field names and generated Python types used by the bridge.
4. Trace every position, velocity, attitude, and timestamp field end to end:
   simulator input, ArduPilot internal frame, MAVLink wire units, pymavlink
   object, and ROS output frame.
5. Record whether an origin is simulator-fixed, EKF-created, home-relative, or
   vehicle-relative. Treat `HOME_POSITION`, EKF origin, and simulator origin as
   distinct until source proves otherwise.
6. Confirm behavior with a bounded Docker probe that prints representative raw
   values. Run `make down` before and after the probe.
7. Add pure conversion tests for units, axes, signs, finite-value rejection, and
   a non-zero origin before changing live bridge behavior.

Report the pinned source paths and commit alongside conclusions so a later
ArduPilot upgrade can repeat the audit.
