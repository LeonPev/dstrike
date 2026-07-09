"""MuJoCo-backed ArduPilot JSON external-physics service."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import socket
import sys
import time
from typing import TextIO

from src.physics.ardupilot_json_stub import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    PacketError,
    PwmPacket,
    parse_pwm_packet,
)
from src.physics.frame_conventions import ardupilot_state_from_quad_state
from src.physics.mujoco_quad import DEFAULT_MODEL_PATH, MuJoCoQuad
from src.physics.ros_camera import CameraConfig, SimCameraPublisher
from src.physics.ros_visualization import SimVisualizationPublisher


GRAVITY_MPS2 = 9.80665
LOG_INTERVAL_SECONDS = 1.0
DEFAULT_PWM_MIN = 1000
DEFAULT_PWM_MAX = 2000
DEFAULT_THRUST_TO_WEIGHT = 2.0
DEFAULT_MAX_FRAME_DT = 0.05


@dataclass(frozen=True)
class StepDecision:
    frame_dt: float
    substeps: int
    reset: bool


@dataclass(frozen=True)
class PwmThrustMapper:
    pwm_min: int
    pwm_max: int
    max_thrust_per_motor: float

    def __post_init__(self) -> None:
        if self.pwm_max <= self.pwm_min:
            raise ValueError("pwm_max must be greater than pwm_min")
        if self.max_thrust_per_motor <= 0.0:
            raise ValueError("max_thrust_per_motor must be positive")

    def thrusts_from_packet(self, packet: PwmPacket) -> tuple[float, float, float, float]:
        if len(packet.pwm) < 4:
            raise ValueError("packet must include at least four PWM channels")

        return tuple(self._channel_to_thrust(value) for value in packet.pwm[:4])

    def _channel_to_thrust(self, pwm_value: int) -> float:
        clipped = min(max(float(pwm_value), float(self.pwm_min)), float(self.pwm_max))
        fraction = (clipped - self.pwm_min) / (self.pwm_max - self.pwm_min)
        return fraction * self.max_thrust_per_motor


class FixedStepClock:
    """Accumulate ArduPilot frame timing into bounded MuJoCo substeps."""

    def __init__(self, fixed_dt: float, max_frame_dt: float) -> None:
        if fixed_dt <= 0.0:
            raise ValueError("fixed_dt must be positive")
        if max_frame_dt < fixed_dt:
            raise ValueError("max_frame_dt must be at least one fixed step")

        self.fixed_dt = fixed_dt
        self.max_frame_dt = max_frame_dt
        self.max_substeps = max(1, math.ceil(max_frame_dt / fixed_dt))
        self.pending_dt = 0.0
        self.last_frame_count: int | None = None

    def consume(self, packet: PwmPacket) -> StepDecision:
        reset = False
        if self.last_frame_count is None:
            frame_delta = 1
        elif packet.frame_count < self.last_frame_count:
            reset = True
            self.pending_dt = 0.0
            frame_delta = 1
        else:
            frame_delta = packet.frame_count - self.last_frame_count

        self.last_frame_count = packet.frame_count
        frame_dt = self._bounded_frame_dt(frame_delta, packet.frame_rate)
        self.pending_dt += frame_dt

        substeps = min(int(self.pending_dt / self.fixed_dt), self.max_substeps)
        if substeps > 0:
            self.pending_dt -= substeps * self.fixed_dt

        self.pending_dt = min(self.pending_dt, self.max_frame_dt)
        return StepDecision(frame_dt=frame_dt, substeps=substeps, reset=reset)

    def _bounded_frame_dt(self, frame_delta: int, frame_rate: int) -> float:
        if frame_delta <= 0 or frame_rate <= 0:
            return 0.0
        return min(frame_delta / float(frame_rate), self.max_frame_dt)


def build_state_json(state: object) -> tuple[dict[str, object], bytes]:
    payload = ardupilot_state_from_quad_state(state)
    encoded = json.dumps(payload, separators=(",", ":"))
    return payload, f"{encoded}\n".encode("utf-8")


def should_log_frame(now: float, last_log_at: float | None) -> bool:
    return last_log_at is None or now - last_log_at >= LOG_INTERVAL_SECONDS


def log_frame(
    packet: PwmPacket,
    step: StepDecision,
    thrusts: tuple[float, float, float, float],
    ned_position: list[float],
    fixed_timestep: float,
    stream: TextIO,
) -> None:
    pwm_first4 = packet.pwm[:4]
    print(
        "mujoco_json "
        f"frame_count={packet.frame_count} "
        f"frame_rate={packet.frame_rate} "
        f"timestep={fixed_timestep:.6f} "
        f"frame_dt={step.frame_dt:.6f} "
        f"substeps={step.substeps} "
        f"pwm_min={min(pwm_first4)} "
        f"pwm_max={max(pwm_first4)} "
        f"thrust_min={min(thrusts):.4f} "
        f"thrust_max={max(thrusts):.4f} "
        f"ned_position=[{ned_position[0]:.4f},{ned_position[1]:.4f},{ned_position[2]:.4f}]",
        file=stream,
        flush=True,
    )


def serve(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    pwm_min: int = DEFAULT_PWM_MIN,
    pwm_max: int = DEFAULT_PWM_MAX,
    thrust_to_weight: float = DEFAULT_THRUST_TO_WEIGHT,
    max_frame_dt: float = DEFAULT_MAX_FRAME_DT,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> None:
    quad = MuJoCoQuad(model_path)
    max_thrust_per_motor = quad.mass * GRAVITY_MPS2 * thrust_to_weight / 4.0
    thrust_mapper = PwmThrustMapper(
        pwm_min=pwm_min,
        pwm_max=pwm_max,
        max_thrust_per_motor=max_thrust_per_motor,
    )
    clock = FixedStepClock(fixed_dt=quad.timestep, max_frame_dt=max_frame_dt)
    ros_visualization = SimVisualizationPublisher(quad)
    camera_config = CameraConfig.from_env()
    camera_publisher = (
        SimCameraPublisher(quad, camera_config) if camera_config.enabled else None
    )
    ros_visualization.publish(quad.state())
    if camera_publisher is not None:
        camera_publisher.publish()
    last_log_at: float | None = None

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.bind((host, port))
            print(
                "MuJoCo JSON physics listening "
                f"udp://{host}:{port} "
                f"model={Path(model_path)} "
                f"fixed_timestep={quad.timestep:.6f} "
                f"max_frame_dt={max_frame_dt:.6f} "
                f"max_thrust_per_motor={max_thrust_per_motor:.4f}",
                file=stdout,
                flush=True,
            )

            while True:
                payload, sender = udp.recvfrom(4096)
                try:
                    packet = parse_pwm_packet(payload)
                    thrusts = thrust_mapper.thrusts_from_packet(packet)
                except (PacketError, ValueError) as exc:
                    print(
                        f"invalid_pwm_packet sender={sender[0]}:{sender[1]} error={exc}",
                        file=stderr,
                        flush=True,
                    )
                    continue

                step = clock.consume(packet)
                if step.reset:
                    quad.reset()
                    print(
                        f"mujoco_json reset frame_count={packet.frame_count}",
                        file=stdout,
                        flush=True,
                    )

                state = (
                    quad.step(thrusts, substeps=step.substeps)
                    if step.substeps
                    else quad.state()
                )
                ros_visualization.maybe_publish(state)
                if camera_publisher is not None:
                    camera_publisher.maybe_publish()
                state_payload, state_json = build_state_json(state)
                udp.sendto(state_json, sender)

                now = time.monotonic()
                if should_log_frame(now, last_log_at):
                    log_frame(
                        packet=packet,
                        step=step,
                        thrusts=thrusts,
                        ned_position=state_payload["position"],  # type: ignore[arg-type]
                        fixed_timestep=quad.timestep,
                        stream=stdout,
                    )
                    last_log_at = now
    finally:
        if camera_publisher is not None:
            camera_publisher.close()
        ros_visualization.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve MuJoCo state through ArduPilot's JSON physics protocol."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP bind host.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP bind port.")
    parser.add_argument(
        "--model-path",
        default=str(DEFAULT_MODEL_PATH),
        help="MuJoCo XML model path.",
    )
    parser.add_argument("--pwm-min", type=int, default=DEFAULT_PWM_MIN)
    parser.add_argument("--pwm-max", type=int, default=DEFAULT_PWM_MAX)
    parser.add_argument(
        "--thrust-to-weight",
        type=float,
        default=DEFAULT_THRUST_TO_WEIGHT,
        help="Total max thrust as a multiple of vehicle weight.",
    )
    parser.add_argument(
        "--max-frame-dt",
        type=float,
        default=DEFAULT_MAX_FRAME_DT,
        help="Clamp ArduPilot frame timing before accumulating fixed substeps.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        serve(
            host=args.host,
            port=args.port,
            model_path=args.model_path,
            pwm_min=args.pwm_min,
            pwm_max=args.pwm_max,
            thrust_to_weight=args.thrust_to_weight,
            max_frame_dt=args.max_frame_dt,
        )
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
