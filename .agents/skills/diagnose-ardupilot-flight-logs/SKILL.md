---
name: diagnose-ardupilot-flight-logs
description: Diagnose ArduPilot SITL takeoff, hover, motion, landing, disarm, and mode-transition failures in dsriker. Use when a live flight or bounded probe fails, flaps between modes, stays armed, or behaves in a way ROS output alone cannot explain. Decode DataFlash .BIN logs inside Docker and correlate them with MAVLink acknowledgements plus ROS and MuJoCo telemetry. Do not use for basic ROS topic checks or non-ArduPilot logs.
---

# Diagnose ArduPilot Flight Logs

## Preserve and select evidence

1. Record the failing probe output. While the stack is still running, inspect
   `docker compose logs --tail 200 ros2` for sent commands and `COMMAND_ACK`
   results.
2. Run `make down` so ArduPilot flushes the current DataFlash log.
3. Read `artifacts/ardupilot/sitl-json/logs/LASTLOG.TXT`. Inspect that numbered
   `.BIN` file; do not guess from modification times or reuse an older log.

## Decode inside Docker

Keep all dependencies inside the project image. Run the parser from an
ephemeral service container, replacing the example filename with the exact
zero-padded number selected above:

```bash
docker compose run --rm --no-deps ros2 bash -lc \
  'python3 /opt/ardupilot/modules/mavlink/pymavlink/tools/mavlogdump.py \
  --types MODE,EV,ERR,MSG,STAT \
  /workspace/artifacts/ardupilot/sitl-json/logs/00000021.BIN'
```

Start with `MODE,EV,ERR,MSG,STAT`. Add only records relevant to the failure:

- Use `ATT,CTUN,POS` for attitude, controller, altitude, and position behavior.
- Use `LDET` for landing-detector state.
- Correlate command `21` with LAND and command `400` with arm/disarm.
- Treat acknowledgement result `0` as accepted; still verify the resulting
  heartbeat mode and armed state.
- Flag rapid or contradictory mode transitions, such as
  `LAND -> GUIDED -> LAND`, and trace every transition back to a command sender.

## Apply safety gates

- Do not recommend force-disarm from ArduPilot estimator state alone.
- Require fresh MuJoCo ground truth showing a stationary, level vehicle on the
  expected terrain or pad contact surface.
- Do not weaken landing, tilt, velocity, or contact thresholds merely to make a
  probe pass. Fix the command sequence, controller, or physical support margin.

## Verify the fix

1. Add a focused unit test for the failed command or state sequence.
2. Run the relevant tests inside Docker.
3. Start a clean stack and rerun the bounded live flight probe.
4. Require the probe PASS line and `armed=false`.
5. Decode the new `.BIN` log and confirm the bad mode or state sequence is gone.
6. Finish with `make down` and verify no Compose services remain.
