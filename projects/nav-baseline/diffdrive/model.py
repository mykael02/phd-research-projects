"""Unicycle (differential-drive) kinematics with command limits and disc collision.

Frame: the same frame as rule G (see geometry.py). x grows with the column
index and y grows with the row index, so in the printed map y points down.
The heading theta is measured from +x towards +y, so a positive turn rate
appears clockwise when the map is printed top row first.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from gridnav import Grid

from .geometry import disc_in_collision

Command = Tuple[float, float]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.remainder(angle, 2.0 * math.pi)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Limits:
    """Inclusive limits: v in [v_min, v_max], omega in [-omega_max, omega_max]."""

    v_min: float
    v_max: float
    omega_max: float

    def __post_init__(self) -> None:
        values = (self.v_min, self.v_max, self.omega_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("limits must be finite")
        if self.v_min > self.v_max:
            raise ValueError("v_min must be <= v_max")
        if self.omega_max < 0:
            raise ValueError("omega_max must be >= 0")


@dataclass(frozen=True)
class ClippedCommand:
    v: float
    omega: float
    clipped: bool


def clip_command(v: float, omega: float, limits: Limits) -> ClippedCommand:
    """Clip (v, omega) to the limits and report whether either value changed.

    Non-finite commands raise ValueError rather than being clipped.
    """
    if not (math.isfinite(v) and math.isfinite(omega)):
        raise ValueError("command must be finite")
    clipped_v = min(max(v, limits.v_min), limits.v_max)
    clipped_omega = min(max(omega, -limits.omega_max), limits.omega_max)
    return ClippedCommand(clipped_v, clipped_omega, clipped_v != v or clipped_omega != omega)


def _check_dt(dt: float) -> None:
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and > 0")


def step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Advance the pose by one fixed step with a constant (v, omega).

    omega == 0 uses the straight-line update; otherwise the exact arc of the
    unicycle model is used. No limits are applied here; see clip_command.
    The returned heading is wrapped to [-pi, pi].
    """
    _check_dt(dt)
    if not (math.isfinite(v) and math.isfinite(omega)):
        raise ValueError("command must be finite")
    if omega == 0:
        distance = v * dt
        return Pose(
            pose.x + distance * math.cos(pose.theta),
            pose.y + distance * math.sin(pose.theta),
            wrap_angle(pose.theta),
        )
    theta_end = pose.theta + omega * dt
    ratio = v / omega
    return Pose(
        pose.x + ratio * (math.sin(theta_end) - math.sin(pose.theta)),
        pose.y - ratio * (math.cos(theta_end) - math.cos(pose.theta)),
        wrap_angle(theta_end),
    )


@dataclass(frozen=True)
class RolloutResult:
    """poses[0] is the start; poses[i] is the pose after step i.

    collision_step is the index into poses of the first pose in collision
    (0 means the start pose itself), or None if no checked pose collided.
    clipped_steps lists the 1-based step indices whose command was clipped.
    """

    poses: Tuple[Pose, ...]
    collision_step: Optional[int]
    clipped_steps: Tuple[int, ...]

    @property
    def collided(self) -> bool:
        return self.collision_step is not None


def rollout(
    grid: Grid,
    start: Pose,
    commands: Sequence[Command],
    dt: float,
    limits: Limits,
    radius: float = 0.0,
) -> RolloutResult:
    """Apply each command for one step of dt, checking the disc footprint.

    The footprint (rule G4) is checked only at the start pose and at the pose
    after each step, never along the motion between steps. The rollout stops at
    the first pose in collision.
    """
    _check_dt(dt)
    if not math.isfinite(radius) or radius < 0:
        raise ValueError("radius must be finite and >= 0")
    poses = [start]
    clipped_steps = []
    if disc_in_collision(grid, (start.x, start.y), radius):
        return RolloutResult(tuple(poses), 0, ())
    pose = start
    for index, (v, omega) in enumerate(commands, start=1):
        command = clip_command(v, omega, limits)
        if command.clipped:
            clipped_steps.append(index)
        pose = step(pose, command.v, command.omega, dt)
        poses.append(pose)
        if disc_in_collision(grid, (pose.x, pose.y), radius):
            return RolloutResult(tuple(poses), index, tuple(clipped_steps))
    return RolloutResult(tuple(poses), None, tuple(clipped_steps))
