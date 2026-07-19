# ArduPilot flight log diagnostics

Use when a live SITL flight fails but ROS output does not identify the mode or
landing-state transition that caused it.

1. Stop the Compose stack so ArduPilot flushes the current `.BIN` log.
2. Read `artifacts/ardupilot/sitl-json/logs/LASTLOG.TXT` and inspect that numbered
   log; do not guess which historical file belongs to the failed run.
3. Keep analysis inside the project image. The parser is
   `/opt/ardupilot/modules/mavlink/pymavlink/tools/mavlogdump.py`.
4. Start with `MODE,EV,ERR,MSG,STAT`, then add only the flight records needed for
   the failure (for example `ATT,CTUN,POS,LDET`).
5. Correlate mode changes with bridge `COMMAND_ACK` records: command `21` is
   LAND and command `400` is arm/disarm.
6. Treat force-disarm as a safety action. Confirm fresh MuJoCo ground truth is
   stationary, level, and on the expected contact surface before recommending
   it.
7. Add a focused unit test for the discovered command sequence and rerun the
   bounded live probe from a clean stack.
