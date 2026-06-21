"""In-house 6-DOF thrust-vector-control (TVC) flight sim.

This is the active-stabilisation model: quaternion attitude, RK4 integration
and a PD/PID gimbal controller. RocketPy cannot model thrust vectoring, so this
sim owns the controlled flight. When the RocketPy engine is available its
trusted aerodynamics (centre of pressure, drag coefficient) are injected via the
``aero`` argument so the steering is simulated against believable forces rather
than the rough analytic fallback below.
"""

from __future__ import annotations

import math

import numpy as np

from motor_curve import cumulative_impulse


def run_tvc_flight(curve, specs, aero=None):
    """Simulate the actively stabilised flight.

    ``aero`` (optional) overrides the analytic aerodynamics with RocketPy-derived
    values: ``{"cpFromBase": m, "drag": Cd, "source": "RocketPy"}``.
    Returns ``{"summary": {...}, "samples": [...], "aeroSource": str}``.
    """
    aero = aero or {}
    dry_mass_without_motor = positive(specs, "dryMass", 1.061)
    motor_dry_mass = positive(specs, "motorDryMass", curve.motor_dry_mass or 0.0422)
    dry_mass = dry_mass_without_motor + motor_dry_mass
    propellant_mass = positive(specs, "motorPropellantMass", curve.propellant_mass_kg or 0.0941)
    radius = positive(specs, "radius", 0.05)
    length = positive(specs, "length", 0.99)
    nose_length = positive(specs, "noseLength", 0.22)
    area = math.pi * radius**2
    rho = 1.225
    gravity = np.array([0.0, -9.80665, 0.0])
    cg_dry = dry_cg(specs, dry_mass_without_motor, motor_dry_mass, length)
    cg_prop = bounded(specs, "motorPosition", 0.05, -length, length)
    launch_cg = ((dry_mass * cg_dry) + (propellant_mass * cg_prop)) / (dry_mass + propellant_mass)

    # Aerodynamics: prefer RocketPy-derived values, fall back to the analytic model.
    if math.isfinite(aero.get("cpFromBase", math.nan)):
        cp = float(aero["cpFromBase"])
        aero_source = aero.get("source", "RocketPy")
    else:
        cp = pressure_center(specs, length, nose_length, radius)
        aero_source = "analytic"
    drag = float(aero["drag"]) if math.isfinite(aero.get("drag", math.nan)) else bounded(specs, "dragCoefficient", 0.75, 0.05, 3.0)

    max_gimbal = math.radians(bounded(specs, "maxGimbalDeg", 8.0, 0.0, 25.0))
    servo_tau = bounded(specs, "servoTau", 0.05, 0.001, 1.0)
    kp = bounded(specs, "kp", 0.8, 0.0, 100.0)
    kd = bounded(specs, "kd", 1.5, 0.0, 100.0)
    ki = bounded(specs, "ki", 0.0, 0.0, 100.0)
    wind = np.array([bounded(specs, "windX", 0.0, -50.0, 50.0), 0.0, bounded(specs, "windZ", 0.0, -50.0, 50.0)])
    dt = bounded(specs, "timeStep", 0.01, 0.001, 0.05)
    max_time = bounded(specs, "maxTime", 120.0, 1.0, 600.0)
    inclination = bounded(specs, "inclination", 85.0, 1.0, 90.0)
    heading = bounded(specs, "heading", 90.0, 0.0, 360.0)
    terminate_on_apogee = bool(specs.get("terminateOnApogee"))

    times = np.array([point[0] for point in curve.points], dtype=float)
    forces = np.array([point[1] for point in curve.points], dtype=float)
    impulse = np.array(cumulative_impulse(curve.points), dtype=float)
    total_impulse = impulse[-1]

    state = np.zeros(15)
    state[6:10] = launch_quaternion(inclination, heading)
    log = []
    integral = np.zeros(3)
    time_value = 0.0
    launched = False

    while time_value < max_time:
        quat = normalize_quaternion(state[6:10])
        up_body = rotate_vector(conjugate(quat), np.array([0.0, 1.0, 0.0]))
        error = np.array([up_body[0], 0.0, up_body[2]])
        integral = np.clip(integral + error * dt, -0.5, 0.5)
        rate = state[10:13]
        thrust, mass, cg, inertia = tvc_properties(time_value, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, radius, length)
        log.append(build_tvc_sample(time_value, state, thrust, mass))
        state = rk4_tvc(state, time_value, dt, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki)
        time_value += dt
        if state[1] > 0.0:
            launched = True
        if not launched and state[1] < 0.0:
            state[1] = 0.0
            state[4] = max(0.0, state[4])
        if launched and terminate_on_apogee and state[4] < 0.0:
            break
        if launched and state[1] < 0.0:
            break

    samples = decimate_samples(log, 500)
    apogee_sample = max(log, key=lambda sample: sample["altitude"])
    burnout_sample = min(log, key=lambda sample: abs(sample["time"] - curve.burn_time))
    return {
        "aeroSource": aero_source,
        "summary": {
            "apogeeM": apogee_sample["altitude"],
            "apogeeTimeS": apogee_sample["time"],
            "dryCgM": cg_dry,
            "launchCgM": launch_cg,
            "cpM": cp,
            # Standard convention: negative when CP is forward of CG (unstable).
            # In from-base coords that is (CG - CP) / diameter.
            "staticMarginCal": (launch_cg - cp) / (radius * 2),
            "burnoutTimeS": burnout_sample["time"],
            "burnoutAltitudeM": burnout_sample["altitude"],
            "burnoutSpeedMS": burnout_sample["speed"],
            "flightTimeS": float(samples[-1]["time"]),
            "maxSpeedMS": max(sample["speed"] for sample in samples),
            "maxGimbalDeg": max(math.hypot(sample["gimbalPitchDeg"], sample["gimbalYawDeg"]) for sample in samples),
            "finalX": samples[-1]["x"],
            "finalY": samples[-1]["y"],
        },
        "samples": samples,
    }


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


def tvc_properties(time_value, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, radius, length):
    thrust = float(np.interp(time_value, times, forces, left=forces[0], right=0.0))
    used_impulse = float(np.interp(time_value, times, impulse, left=0.0, right=total_impulse))
    propellant = propellant_mass * max(0.0, 1.0 - used_impulse / total_impulse) if total_impulse > 0 else 0.0
    mass = dry_mass + propellant
    cg = ((dry_mass * cg_dry) + (propellant * cg_prop)) / mass
    i_roll = 0.5 * mass * radius**2
    i_pitch = (1 / 12) * mass * length**2 + 0.25 * mass * radius**2
    return thrust, mass, cg, np.diag([i_pitch, i_roll, i_pitch])


def rk4_tvc(state, time_value, dt, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki):
    k1 = tvc_derivatives(state, time_value, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki)
    k2 = tvc_derivatives(state + 0.5 * dt * k1, time_value + 0.5 * dt, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki)
    k3 = tvc_derivatives(state + 0.5 * dt * k2, time_value + 0.5 * dt, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki)
    k4 = tvc_derivatives(state + dt * k3, time_value + dt, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki)
    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


def tvc_derivatives(state, time_value, error, rate, integral, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, cp, radius, length, area, drag, rho, gravity, wind, max_gimbal, servo_tau, kp, kd, ki):
    velocity = state[3:6]
    quaternion = normalize_quaternion(state[6:10])
    rates = state[10:13]
    gimbal = state[13:15]
    thrust, mass, cg, inertia = tvc_properties(time_value, times, forces, impulse, total_impulse, dry_mass, propellant_mass, cg_dry, cg_prop, radius, length)
    target_pitch = np.clip(-(kp * error[2] + kd * rate[0] + ki * integral[2]), -max_gimbal, max_gimbal)
    target_yaw = np.clip(kp * error[0] + kd * rate[2] + ki * integral[0], -max_gimbal, max_gimbal)
    gimbal_rate = np.array([(target_pitch - gimbal[0]) / servo_tau, (target_yaw - gimbal[1]) / servo_tau])
    thrust_body = np.array([
        -thrust * math.sin(gimbal[1]),
        thrust * math.cos(gimbal[0]) * math.cos(gimbal[1]),
        -thrust * math.sin(gimbal[0]),
    ])
    relative_velocity = velocity - wind
    relative_speed = float(np.linalg.norm(relative_velocity))
    drag_force = np.zeros(3)
    if relative_speed > 1e-9:
        drag_force = -relative_velocity / relative_speed * 0.5 * rho * drag * area * relative_speed**2
    thrust_world = rotate_vector(quaternion, thrust_body)
    acceleration = (thrust_world + drag_force + mass * gravity) / mass
    torque_tvc = np.cross(np.array([0.0, -cg, 0.0]), thrust_body)
    relative_body = rotate_vector(conjugate(quaternion), relative_velocity)
    lateral = np.array([relative_body[0], 0.0, relative_body[2]])
    torque_aero = np.cross(np.array([0.0, cp - cg, 0.0]), -0.5 * rho * drag * area * lateral * relative_speed)
    angular_accel = np.linalg.inv(inertia) @ (torque_tvc + torque_aero - np.cross(rates, inertia @ rates))
    dq = 0.5 * quaternion_multiply(quaternion, np.array([0.0, rates[0], rates[1], rates[2]]))
    return np.concatenate([velocity, acceleration, dq, angular_accel, gimbal_rate])


def build_tvc_sample(time_value, state, thrust, mass):
    speed = math.sqrt(state[3] ** 2 + state[4] ** 2 + state[5] ** 2)
    return {
        "time": time_value,
        "x": float(state[0]),
        "y": float(state[2]),
        "altitude": max(0.0, float(state[1])),
        "speed": speed,
        "thrust": thrust,
        "mass": mass,
        "gimbalPitchDeg": math.degrees(state[13]),
        "gimbalYawDeg": math.degrees(state[14]),
    }


def decimate_samples(samples, limit):
    if len(samples) <= limit:
        return samples
    step = max(1, len(samples) // limit)
    decimated = samples[::step]
    if decimated[-1] is not samples[-1]:
        decimated.append(samples[-1])
    return decimated


def launch_quaternion(inclination_deg, heading_deg):
    """Quaternion that tilts the body +Y axis to the launch direction.

    ``inclination`` is measured from horizontal (90 deg = straight up), matching
    RocketPy's Flight convention, so both engines leave the pad at the same angle.
    """
    tilt = math.radians(90.0 - inclination_deg)
    if abs(tilt) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    heading = math.radians(heading_deg)
    direction = np.array([
        math.sin(tilt) * math.sin(heading),
        math.cos(tilt),
        math.sin(tilt) * math.cos(heading),
    ])
    up = np.array([0.0, 1.0, 0.0])
    axis = np.cross(up, direction)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / axis_norm
    angle = math.acos(float(np.clip(np.dot(up, direction), -1.0, 1.0)))
    half = angle / 2.0
    return np.array([math.cos(half), axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half)])


def quaternion_multiply(first, second):
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def conjugate(quaternion):
    return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])


def rotate_vector(quaternion, vector):
    q_vector = np.array([0.0, vector[0], vector[1], vector[2]])
    return quaternion_multiply(quaternion_multiply(quaternion, q_vector), conjugate(quaternion))[1:]


def normalize_quaternion(quaternion):
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


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
