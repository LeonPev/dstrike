# ArduPilot JSON Dual-SITL Routing

Use this skill when adding or diagnosing multiple ArduPilot SITL instances that use the JSON external-physics backend.

## Pinned backend contract

- The model argument selects only the destination host: `--model JSON:<physics-ip>`.
- Do not append a UDP port to the model string. In the pinned ArduPilot backend, everything after the first colon is treated as the host string.
- Select the physics destination port with the SITL binary argument `--sim-port-out=<port>`.
- When launching through `sim_vehicle.py`, pass binary arguments through one quoted `--sitl-instance-args` value, for example:

  ```bash
  --model "JSON:${PHYSICS_IP}" \
  --sitl-instance-args="--sim-port-out=9003 --serial0=udpclient:${ROS2_IP}:14551"
  ```

- Give every vehicle a unique `SYSID_THISMAV`, MAVLink destination port, physics destination port, artifact directory, and ROS namespace.

## Verification

Inspect SITL startup output. A correct target route includes:

```text
JSON control interface set to <physics-ip>:9003
```

If SITL repeatedly prints `No JSON sensor message received`, compare the logged control-interface destination with the physics listener and inspect the pinned implementation at `/opt/ardupilot/libraries/SITL/SIM_JSON.cpp` inside the project image.
