"""In-house 6-DOF thrust-vector-control (TVC) flight sim.

This is the active-stabilisation model: quaternion attitude, RK4 integration of
the rigid-body physics, and a realistic flight computer in the loop (see
``flight_computer.py``). The gimbal is NOT driven by perfect truth - it is driven
by a modelled MPU6050 -> attitude estimator -> discrete PID -> servo chain, with
sensor noise/bias, on-pad calibration, control-loop latency and servo slew/limit.

RocketPy cannot model thrust vectoring, so this sim owns the controlled flight.
When the RocketPy engine is available its trusted aerodynamics (centre of
pressure, drag coefficient) are injected via the ``aero`` argument so the
steering is simulated against believable forces rather than the rough analytic
fallback below.

Physics state integrated by RK4 is 13-dimensional:
``[pos(3), vel(3), quat(4), rates(3)]`` in the world frame (Y up). The gimbal is
an external input held (zero-order hold) across each physics sub-step and updated
by the flight computer at its control rate.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

from flight_computer import FlightComputer, params_from_specs
from motor_curve import cumulative_impulse
from quaternions import (
    launch_quaternion,
    normalize_quaternion,
    quaternion_multiply,
    quaternion_to_matrix,
    tilt_from_vertical_deg,
)

GRAVITY = 9.80665


def run_tvc_flight(curve, specs, aero=None):
    """Simulate the actively stabilised flight with a flight computer in the loop.

    ``aero`` (optional) overrides the analytic aerodynamics with RocketPy-derived
    values: ``{"cpFromBase": m, "drag": Cd, "source": "RocketPy"}``.
    Returns ``{"summary": {...}, "samples": [...], "aeroSource": str}``.
    """
    aero = aero or {}
    ctx, geometry = _build_context(curve, specs, aero)

    fc_params = params_from_specs(specs, bounded)
    seed = int(bounded(specs, "seed", 0.0, 0.0, 2**31 - 1)) if specs.get("seed") is not None else 0
    rng = np.random.default_rng(seed)
    fc = FlightComputer(fc_params, rng)

    inclination = bounded(specs, "inclination", 85.0, 1.0, 90.0)
    heading = bounded(specs, "heading", 90.0, 0.0, 360.0)
    launch_quat = launch_quaternion(inclination, heading)
    fc.calibrate(launch_quat)

    dt_request = bounded(specs, "timeStep", 0.01, 0.001, 0.05)
    phys_dt_target = min(dt_request, 0.005)  # ~200 Hz physics is ample for this airframe
    substeps = max(1, int(round(fc.control_dt / phys_dt_target)))
    phys_dt = fc.control_dt / substeps
    max_time = bounded(specs, "maxTime", 120.0, 1.0, 600.0)
    terminate_on_apogee = bool(specs.get("terminateOnApogee"))

    state = np.zeros(13)
    state[6:10] = launch_quat
    gimbal = np.zeros(2)
    time_value = 0.0
    launched = False
    log = []

    while time_value < max_time:
        specific_force_body, thrust_mag, mass = _control_inputs(state, gimbal, time_value, ctx)
        fc.control_tick(state[10:13], specific_force_body)
        log.append(_sample(time_value, state, gimbal, thrust_mag, mass, fc.q_hat))

        landed = False
        for _ in range(substeps):
            gimbal = fc.actuate(phys_dt)
            state = _rk4(state, time_value, phys_dt, gimbal, ctx)
            time_value += phys_dt
            if not np.all(np.isfinite(state)):
                landed = True  # numerical divergence: stop integrating
                break
            if state[1] > 0.0:
                launched = True
            if not launched and state[1] < 0.0:
                state[1] = 0.0
                state[4] = max(0.0, state[4])
            if launched and state[1] < 0.0:
                landed = True
                break
        if landed:
            break
        if terminate_on_apogee and launched and state[4] < 0.0:
            break

    samples = decimate_samples(log, 500)
    apogee_sample = max(log, key=lambda sample: sample["altitude"])
    burnout_sample = min(log, key=lambda sample: abs(sample["time"] - curve.burn_time))
    # TVC only has authority while the motor burns; after burnout an unstable
    # finless rocket tumbles freely, so stability is judged over the powered phase.
    powered = [sample for sample in log if sample["time"] <= curve.burn_time]
    max_tilt_powered = max((sample["tiltDeg"] for sample in powered), default=0.0)
    # Estimator error over the powered phase (after burnout the gyro saturates in
    # the tumble, so the whole-flight error is not a meaningful sensor metric).
    max_est_error_powered = max((abs(sample["tiltDeg"] - sample["estTiltDeg"]) for sample in powered), default=0.0)
    aero_source = aero.get("source", "RocketPy") if math.isfinite(aero.get("cpFromBase", math.nan)) else "analytic"
    return {
        "aeroSource": aero_source,
        "summary": {
            "apogeeM": apogee_sample["altitude"],
            "apogeeTimeS": apogee_sample["time"],
            "dryCgM": geometry["cg_dry"],
            "launchCgM": geometry["launch_cg"],
            "cpM": ctx.cp,
            # Standard convention: negative when CP is forward of CG (unstable).
            "staticMarginCal": (geometry["launch_cg"] - ctx.cp) / (ctx.radius * 2),
            "burnoutTimeS": burnout_sample["time"],
            "burnoutAltitudeM": burnout_sample["altitude"],
            "burnoutSpeedMS": burnout_sample["speed"],
            "flightTimeS": float(samples[-1]["time"]),
            "maxSpeedMS": max(sample["speed"] for sample in samples),
            "maxGimbalDeg": max(math.hypot(sample["gimbalPitchDeg"], sample["gimbalYawDeg"]) for sample in samples),
            "maxTiltDeg": max(sample["tiltDeg"] for sample in log),
            "maxTiltPoweredDeg": max_tilt_powered,
            "tiltAtBurnoutDeg": burnout_sample["tiltDeg"],
            "maxEstErrorDeg": max(abs(sample["tiltDeg"] - sample["estTiltDeg"]) for sample in log),
            "maxEstErrorPoweredDeg": max_est_error_powered,
            "gyroBiasDps": fc.gyro_bias_dps,
            "biasResidualDps": fc.bias_residual_dps,
            "controlRateHz": fc_params.control_rate_hz,
            "loopLatencyMs": fc_params.loop_latency_s * 1000.0,
            "finalX": samples[-1]["x"],
            "finalY": samples[-1]["y"],
        },
        "samples": samples,
    }


def _build_context(curve, specs, aero):
    dry_mass_without_motor = positive(specs, "dryMass", 1.061)
    motor_dry_mass = positive(specs, "motorDryMass", curve.motor_dry_mass or 0.0422)
    dry_mass = dry_mass_without_motor + motor_dry_mass
    propellant_mass = positive(specs, "motorPropellantMass", curve.propellant_mass_kg or 0.0941)
    radius = positive(specs, "radius", 0.05)
    length = positive(specs, "length", 0.99)
    nose_length = positive(specs, "noseLength", 0.22)
    area = math.pi * radius**2
    cg_dry = dry_cg(specs, dry_mass_without_motor, motor_dry_mass, length)
    cg_prop = bounded(specs, "motorPosition", 0.05, -length, length)
    launch_cg = ((dry_mass * cg_dry) + (propellant_mass * cg_prop)) / (dry_mass + propellant_mass)

    if math.isfinite(aero.get("cpFromBase", math.nan)):
        cp = float(aero["cpFromBase"])
    else:
        cp = pressure_center(specs, length, nose_length, radius)
    drag = float(aero["drag"]) if math.isfinite(aero.get("drag", math.nan)) else bounded(specs, "dragCoefficient", 0.75, 0.05, 3.0)

    times = np.array([point[0] for point in curve.points], dtype=float)
    forces = np.array([point[1] for point in curve.points], dtype=float)
    impulse = np.array(cumulative_impulse(curve.points), dtype=float)

    ctx = SimpleNamespace(
        times=times,
        forces=forces,
        impulse=impulse,
        total_impulse=impulse[-1],
        dry_mass=dry_mass,
        propellant_mass=propellant_mass,
        cg_dry=cg_dry,
        cg_prop=cg_prop,
        radius=radius,
        length=length,
        cp=cp,
        area=area,
        drag=drag,
        rho=1.225,
        gravity=np.array([0.0, -GRAVITY, 0.0]),
        wind=np.array([bounded(specs, "windX", 0.0, -50.0, 50.0), 0.0, bounded(specs, "windZ", 0.0, -50.0, 50.0)]),
        # thrust misalignment disturbance (rad), used by Monte Carlo robustness runs
        misalign=np.array([
            bounded(specs, "thrustMisalignPitchRad", 0.0, -0.2, 0.2),
            bounded(specs, "thrustMisalignYawRad", 0.0, -0.2, 0.2),
        ]),
    )
    geometry = {"cg_dry": cg_dry, "launch_cg": launch_cg}
    return ctx, geometry


def _thrust_body(thrust_mag, gimbal, ctx):
    gimbal_pitch = gimbal[0] + ctx.misalign[0]
    gimbal_yaw = gimbal[1] + ctx.misalign[1]
    return np.array([
        -thrust_mag * math.sin(gimbal_yaw),
        thrust_mag * math.cos(gimbal_pitch) * math.cos(gimbal_yaw),
        -thrust_mag * math.sin(gimbal_pitch),
    ])


def _control_inputs(state, gimbal, time_value, ctx):
    """Specific force the accelerometer feels (body frame) plus thrust/mass for logging."""
    quaternion = normalize_quaternion(state[6:10])
    rotation = quaternion_to_matrix(quaternion)
    thrust_mag, mass, _, _, _ = tvc_properties(
        time_value, ctx.times, ctx.forces, ctx.impulse, ctx.total_impulse,
        ctx.dry_mass, ctx.propellant_mass, ctx.cg_dry, ctx.cg_prop, ctx.radius, ctx.length,
    )
    relative = state[3:6] - ctx.wind
    speed = float(np.linalg.norm(relative))
    drag_world = -relative / speed * (0.5 * ctx.rho * ctx.drag * ctx.area * speed * speed) if speed > 1e-9 else np.zeros(3)
    specific_force_world = (rotation @ _thrust_body(thrust_mag, gimbal, ctx) + drag_world) / mass
    return rotation.T @ specific_force_world, thrust_mag, mass


def _derivatives(state, time_value, gimbal, ctx):
    quaternion = normalize_quaternion(state[6:10])
    rotation = quaternion_to_matrix(quaternion)
    rates = state[10:13]
    thrust_mag, mass, cg, i_pitch, i_roll = tvc_properties(
        time_value, ctx.times, ctx.forces, ctx.impulse, ctx.total_impulse,
        ctx.dry_mass, ctx.propellant_mass, ctx.cg_dry, ctx.cg_prop, ctx.radius, ctx.length,
    )
    thrust_body = _thrust_body(thrust_mag, gimbal, ctx)
    thrust_world = rotation @ thrust_body

    relative = state[3:6] - ctx.wind
    speed = float(np.linalg.norm(relative))
    drag_world = -relative / speed * (0.5 * ctx.rho * ctx.drag * ctx.area * speed * speed) if speed > 1e-9 else np.zeros(3)
    acceleration = (thrust_world + drag_world + mass * ctx.gravity) / mass

    torque_tvc = np.cross(np.array([0.0, -cg, 0.0]), thrust_body)
    relative_body = rotation.T @ relative
    lateral = np.array([relative_body[0], 0.0, relative_body[2]])
    aero_force = -0.5 * ctx.rho * ctx.drag * ctx.area * lateral * speed
    torque_aero = np.cross(np.array([0.0, ctx.cp - cg, 0.0]), aero_force)

    # Diagonal inertia tensor diag(i_pitch, i_roll, i_pitch): invert by reciprocal.
    inertia_omega = np.array([i_pitch * rates[0], i_roll * rates[1], i_pitch * rates[2]])
    net_torque = torque_tvc + torque_aero - np.cross(rates, inertia_omega)
    angular_accel = np.array([net_torque[0] / i_pitch, net_torque[1] / i_roll, net_torque[2] / i_pitch])

    dq = 0.5 * quaternion_multiply(quaternion, np.array([0.0, rates[0], rates[1], rates[2]]))
    return np.concatenate([state[3:6], acceleration, dq, angular_accel])


def _rk4(state, time_value, dt, gimbal, ctx):
    k1 = _derivatives(state, time_value, gimbal, ctx)
    k2 = _derivatives(state + 0.5 * dt * k1, time_value + 0.5 * dt, gimbal, ctx)
    k3 = _derivatives(state + 0.5 * dt * k2, time_value + 0.5 * dt, gimbal, ctx)
    k4 = _derivatives(state + dt * k3, time_value + dt, gimbal, ctx)
    new_state = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    new_state[6:10] = normalize_quaternion(new_state[6:10])
    return new_state


def _sample(time_value, state, gimbal, thrust, mass, q_hat):
    speed = math.sqrt(state[3] ** 2 + state[4] ** 2 + state[5] ** 2)
    return {
        "time": time_value,
        "x": float(state[0]),
        "y": float(state[2]),
        "altitude": max(0.0, float(state[1])),
        "speed": speed,
        "thrust": thrust,
        "mass": mass,
        "gimbalPitchDeg": math.degrees(gimbal[0]),
        "gimbalYawDeg": math.degrees(gimbal[1]),
        "tiltDeg": tilt_from_vertical_deg(normalize_quaternion(state[6:10])),
        "estTiltDeg": tilt_from_vertical_deg(normalize_quaternion(q_hat)),
    }


def decimate_samples(samples, limit):
    if len(samples) <= limit:
        return samples
    step = max(1, len(samples) // limit)
    decimated = samples[::step]
    if decimated[-1] is not samples[-1]:
        decimated.append(samples[-1])
    return decimated


def tvc_properties(time_value, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, radius, length):
    thrust = float(np.interp(time_value, times, forces, left=forces[0], right=0.0))
    used_impulse = float(np.interp(time_value, times, impulse, left=0.0, right=total_impulse))
    propellant = propellant_mass * max(0.0, 1.0 - used_impulse / total_impulse) if total_impulse > 0 else 0.0
    mass = dry_mass + propellant
    cg = ((dry_mass * cg_dry) + (propellant * cg_prop)) / mass
    i_roll = 0.5 * mass * radius**2
    i_pitch = (1 / 12) * mass * length**2 + 0.25 * mass * radius**2
    return thrust, mass, cg, i_pitch, i_roll


def dry_cg(specs, airframe_mass, motor_dry_mass, length):
    components = specs.get("components")
    if not isinstance(components, list) or not components:
        return bounded(specs, "centerOfMass", 0.35, -length, length)
    total_mass = 0.0
    moment = 0.0
    for component in components[:24]:
        if not isinstance(component, dict):
            raise ValueError("CG components must be objects.")
        mass = positive(component, "mass", 0.0)
        position = bounded(component, "position", 0.0, -length, length * 2)
        total_mass += mass
        moment += mass * position
    motor_position = bounded(specs, "motorPosition", 0.05, -length, length)
    total_mass += motor_dry_mass
    moment += motor_dry_mass * motor_position
    return moment / total_mass


def pressure_center(specs, length, nose_length, radius):
    """Rough finless CP estimate; used only when RocketPy is unavailable."""
    if specs.get("useManualCp"):
        return bounded(specs, "centerOfPressure", 0.60, -length, length * 2)
    diameter = radius * 2
    nose_tip = length
    nose_base = length - nose_length
    nose_cp = nose_tip - nose_length * nose_cp_factor(str(specs.get("noseShape", "vonKarman")))
    body_cp = nose_base / 2
    body_normal = max(0.0, 1.1 * (diameter / max(length - nose_length, diameter)) ** 2)
    nose_normal = 2.0
    return ((nose_normal * nose_cp) + (body_normal * body_cp)) / (nose_normal + body_normal)


def nose_cp_factor(shape):
    values = {
        "cone": 2 / 3,
        "ogive": 0.466,
        "vonKarman": 0.5,
        "elliptical": 0.5,
    }
    return values.get(shape, values["vonKarman"])


def positive(specs, key, default):
    value = float(specs.get(key, default))
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{key} must be positive.")
    return value


def bounded(specs, key, default, minimum, maximum):
    value = float(specs.get(key, default))
    if not math.isfinite(value) or value < minimum or value > maximum:
        raise ValueError(f"{key} must be between {minimum} and {maximum}.")
    return value
