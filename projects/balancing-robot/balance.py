"""Planar wheeled inverted pendulum; SI units and total two-motor torque."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Parameters:
    base_mass: float = 0.35
    body_mass: float = 1.2
    length: float = 0.18
    body_inertia: float = 0.012
    wheel_inertia: float = 0.00012  # per wheel, about its axle
    radius: float = 0.045
    gravity: float = 9.81
    translation_drag: float = 0.08
    rotation_drag: float = 0.002
    torque_limit: float = 0.7

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ("base_mass", "body_mass", "length", "body_inertia", "radius", "gravity", "torque_limit"):
            if getattr(self, key) == 0:
                raise ValueError(f"{key} must be positive")


def clamp(value, limit):
    return max(-limit, min(limit, value))


def derivative(state, torque, push=0.0, p=Parameters()):
    """State [x, velocity, forward tilt from vertical, angular velocity].

    A horizontal external push acts at the body centre of mass. Motor reaction
    torque acts on the body with the opposite sign to wheel torque.
    """
    _, velocity, theta, omega = state
    sin, cos = math.sin(theta), math.cos(theta)
    a = p.base_mass + p.body_mass + 2*p.wheel_inertia/p.radius**2
    b = p.body_mass*p.length*cos
    d = p.body_inertia + p.body_mass*p.length**2
    f = torque/p.radius - p.translation_drag*velocity + p.body_mass*p.length*sin*omega**2 + push
    h = p.body_mass*p.gravity*p.length*sin - torque - p.rotation_drag*omega + push*p.length*cos
    determinant = a*d-b*b
    return (velocity, (d*f-b*h)/determinant, omega, (a*h-b*f)/determinant)


def rk4(state, torque, push, dt, p=Parameters()):
    if not math.isfinite(dt) or not 0 < dt <= 0.02:
        raise ValueError("dt must lie in (0, 0.02]")
    k1 = derivative(state, torque, push, p)
    k2 = derivative(tuple(y+dt*k/2 for y, k in zip(state, k1)), torque, push, p)
    k3 = derivative(tuple(y+dt*k/2 for y, k in zip(state, k2)), torque, push, p)
    k4 = derivative(tuple(y+dt*k for y, k in zip(state, k3)), torque, push, p)
    return tuple(y+dt*(a+2*b+2*c+d)/6 for y, a, b, c, d in zip(state, k1, k2, k3, k4))


def energy(state, p=Parameters()):
    _, v, theta, omega = state
    a = p.base_mass + p.body_mass + 2*p.wheel_inertia/p.radius**2
    d = p.body_inertia+p.body_mass*p.length**2
    return .5*a*v*v + p.body_mass*p.length*math.cos(theta)*v*omega + .5*d*omega*omega + p.body_mass*p.gravity*p.length*math.cos(theta)


class Controller:
    def __init__(self, name, config):
        if name not in ("pid", "lqr", "off"):
            raise ValueError("unknown controller")
        self.name, self.config, self.integral = name, config, 0.0

    def command(self, state, dt, limit):
        x, velocity, theta, omega = state
        if self.name == "off":
            return 0.0, False
        if self.name == "lqr":
            raw = -sum(k*y for k, y in zip(self.config["lqr_gain"], state))
        else:
            gains = self.config["pid"]
            candidate = clamp(self.integral + theta*dt, gains["integral_limit"])
            other = gains["kp"]*theta + gains["kd"]*omega + gains["kx"]*x + gains["kv"]*velocity
            trial = other + gains["ki"]*candidate
            # Conditional integration: accept only when unsaturated or unwinding.
            if abs(trial) <= limit or trial*theta < 0:
                self.integral = candidate
            raw = other + gains["ki"]*self.integral
        return clamp(raw, limit), abs(raw) > limit


def simulate(name, config, initial_degrees=5, push_force=0, push_time=2.0,
             push_duration=.2, duration=10.0, dt=.005, params=None, physics_substeps=1):
    if not all(math.isfinite(v) for v in (initial_degrees, push_force, push_time, push_duration, duration, dt)):
        raise ValueError("simulation inputs must be finite")
    if duration <= 0 or push_time < 0 or push_duration < 0 or not 0 < dt <= .02:
        raise ValueError("invalid time settings")
    if isinstance(physics_substeps, bool) or not isinstance(physics_substeps, int) or physics_substeps < 1:
        raise ValueError("physics_substeps must be a positive integer")
    p = params or Parameters(**config["parameters"])
    control = Controller(name, config)
    state = (0.0, 0.0, math.radians(initial_degrees), 0.0)
    rows, fallen = [], abs(state[2]) >= math.pi/4
    t, effort, saturated, max_x, peak = 0.0, 0.0, 0, 0.0, abs(state[2])
    while t < duration-1e-12 and not fallen:
        step = min(dt, duration-t)
        push = push_force if push_time-1e-10 <= t < push_time+push_duration-1e-10 else 0.0
        torque, clipped = control.command(state, step, p.torque_limit)
        rows.append({"time":t,"state":list(state),"torque":torque,"push":push})
        # Smaller integration steps preserve the controller's sample-and-hold rate.
        for _ in range(physics_substeps):
            state = rk4(state, torque, push, step/physics_substeps, p)
        t += step
        effort += torque*torque*step
        saturated += int(clipped)
        peak, max_x = max(peak, abs(state[2])), max(max_x, abs(state[0]))
        fallen = abs(state[2]) >= math.pi/4
    rows.append({"time":t,"state":list(state),"torque":0.0,"push":0.0})
    # Settling requires the final one second to remain within all three bounds.
    last_bad = None
    reference_time = push_time+push_duration if push_force != 0 else 0.0
    for row in rows:
        x, velocity, theta, omega = row["state"]
        if row["time"] >= reference_time and (abs(theta)>math.radians(1) or abs(x)>.03 or abs(velocity)>.03):
            last_bad = row["time"]
    settled_at = max(reference_time, (last_bad+dt) if last_bad is not None else reference_time)
    recovery = settled_at-reference_time if not fallen and t-settled_at >= 1.0 else None
    metrics = {"controller":name,"fallen":fallen,"duration_s":round(t,6),
               "peak_tilt_deg":round(math.degrees(peak),6),"max_displacement_m":round(max_x,6),
               "final_position_m":round(state[0],6),"final_tilt_deg":round(math.degrees(state[2]),6),
               "recovery_s":None if recovery is None else round(recovery,6),
               "torque_squared_integral":round(effort,6),"saturated_steps":saturated}
    return metrics, rows
