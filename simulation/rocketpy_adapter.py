from __future__ import annotations

import csv
import io
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


MAX_CURVE_BYTES = 1_000_000
MAX_POINTS = 5_000


@dataclass(frozen=True)
class MotorCurve:
    name: str
    points: list[tuple[float, float]]
    diameter_mm: float | None = None
    length_mm: float | None = None
    propellant_mass_kg: float | None = None
    total_mass_kg: float | None = None

    @property
    def burn_time(self):
        return self.points[-1][0]

    @property
    def total_impulse(self):
        impulse = 0.0
        for index in range(1, len(self.points)):
            t0, f0 = self.points[index - 1]
            t1, f1 = self.points[index]
            impulse += 0.5 * (f0 + f1) * (t1 - t0)
        return impulse

    @property
    def average_thrust(self):
        return self.total_impulse / self.burn_time if self.burn_time > 0 else 0.0

    @property
    def peak_thrust(self):
        return max(force for _, force in self.points)


def parse_motor_curve(file_name, content):
    if len(content.encode("utf-8", errors="ignore")) > MAX_CURVE_BYTES:
        raise ValueError("Motor file is too large.")
    suffix = Path(file_name).suffix.lower()
    if suffix == ".rse":
        curve = parse_rse(content)
    elif suffix == ".csv":
        curve = parse_csv_curve(file_name, content)
    else:
        curve = parse_eng(file_name, content)
    return validate_curve(curve)


def parse_eng(file_name, content):
    lines = []
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if line and not line.startswith(";"):
            lines.append(line)
    if len(lines) < 3:
        raise ValueError("RASP .eng file needs a motor header and at least two thrust points.")
    header = lines[0].split()
    if len(header) < 7:
        raise ValueError("RASP .eng header is missing fields.")
    points = []
    for line in lines[1:]:
        parts = line.replace(",", " ").split()
        if len(parts) < 2:
            continue
        points.append((parse_float(parts[0], "time"), parse_float(parts[1], "thrust")))
    return MotorCurve(
        name=header[0] or Path(file_name).stem,
        points=points,
        diameter_mm=parse_float(header[1], "diameter"),
        length_mm=parse_float(header[2], "length"),
        propellant_mass_kg=parse_float(header[4], "propellant mass"),
        total_mass_kg=parse_float(header[5], "total mass"),
    )


def parse_rse(content):
    try:
        root = ET.fromstring(content)
    except ET.ParseError as error:
        raise ValueError(f"Invalid RockSim .rse XML: {error}") from error
    engine = root.find(".//engine")
    if engine is None:
        raise ValueError("RockSim .rse file does not contain an engine.")
    points = []
    for point in engine.findall(".//eng-data"):
        time_value = point.attrib.get("t")
        force_value = point.attrib.get("f")
        if time_value is not None and force_value is not None:
            points.append((parse_float(time_value, "time"), parse_float(force_value, "thrust")))
    if not points:
        for point in engine.findall(".//data"):
            time_value = point.attrib.get("t")
            force_value = point.attrib.get("f")
            if time_value is not None and force_value is not None:
                points.append((parse_float(time_value, "time"), parse_float(force_value, "thrust")))
    return MotorCurve(
        name=engine.attrib.get("code") or engine.attrib.get("mfg") or "uploaded_motor",
        points=points,
        diameter_mm=optional_float(engine.attrib.get("dia"), "diameter"),
        length_mm=optional_float(engine.attrib.get("len"), "length"),
        propellant_mass_kg=grams_to_kg(engine.attrib.get("initWt"), engine.attrib.get("propWt")),
        total_mass_kg=optional_grams_to_kg(engine.attrib.get("initWt")),
    )


def parse_csv_curve(file_name, content):
    reader = csv.reader(io.StringIO(content))
    points = []
    for row in reader:
        if len(row) < 2:
            continue
        try:
            points.append((parse_float(row[0], "time"), parse_float(row[1], "thrust")))
        except ValueError:
            if points:
                raise
    return MotorCurve(name=Path(file_name).stem, points=points)


def validate_curve(curve):
    if len(curve.points) < 2:
        raise ValueError("Motor curve needs at least two thrust points.")
    if len(curve.points) > MAX_POINTS:
        raise ValueError("Motor curve has too many thrust points.")
    points = sorted(curve.points)
    if points[0][0] < 0:
        raise ValueError("Motor curve time cannot be negative.")
    previous_time = -1.0
    for time_value, force_value in points:
        if not math.isfinite(time_value) or not math.isfinite(force_value):
            raise ValueError("Motor curve contains non-finite values.")
        if force_value < 0:
            raise ValueError("Motor curve thrust cannot be negative.")
        if time_value <= previous_time:
            raise ValueError("Motor curve times must be strictly increasing.")
        previous_time = time_value
    if points[0][0] > 0:
        points.insert(0, (0.0, 0.0))
    if points[-1][1] != 0:
        points.append((points[-1][0] + 0.01, 0.0))
    return MotorCurve(
        name=curve.name[:80],
        points=points,
        diameter_mm=curve.diameter_mm,
        length_mm=curve.length_mm,
        propellant_mass_kg=curve.propellant_mass_kg,
        total_mass_kg=curve.total_mass_kg,
    )


def run_simulation(curve, specs):
    import numpy as np
    dry_mass_without_motor = positive(specs, "dryMass", 1.061)
    motor_dry_mass = positive(specs, "motorDryMass", inferred_motor_dry_mass(curve))
    dry_mass = dry_mass_without_motor + motor_dry_mass
    propellant_mass = positive(specs, "motorPropellantMass", curve.propellant_mass_kg or 0.0941)
    radius = positive(specs, "radius", 0.05)
    length = positive(specs, "length", 0.99)
    nose_length = positive(specs, "noseLength", 0.22)
    area = math.pi * radius**2
    drag = bounded(specs, "dragCoefficient", 0.75, 0.05, 3.0)
    rho = 1.225
    gravity = np.array([0.0, -9.80665, 0.0])
    cg_dry = dry_cg(specs, dry_mass_without_motor, motor_dry_mass, length)
    cg_prop = bounded(specs, "motorPosition", 0.05, -length, length)
    cp = pressure_center(specs, length, nose_length, radius)
    launch_cg = ((dry_mass * cg_dry) + (propellant_mass * cg_prop)) / (dry_mass + propellant_mass)
    max_gimbal = math.radians(bounded(specs, "maxGimbalDeg", 8.0, 0.0, 25.0))
    servo_tau = bounded(specs, "servoTau", 0.05, 0.001, 1.0)
    kp = bounded(specs, "kp", 0.8, 0.0, 100.0)
    kd = bounded(specs, "kd", 1.5, 0.0, 100.0)
    ki = bounded(specs, "ki", 0.0, 0.0, 100.0)
    wind = np.array([bounded(specs, "windX", 0.0, -50.0, 50.0), 0.0, bounded(specs, "windZ", 0.0, -50.0, 50.0)])
    dt = bounded(specs, "timeStep", 0.01, 0.001, 0.05)
    max_time = bounded(specs, "maxTime", 120.0, 1.0, 600.0)
    times = np.array([point[0] for point in curve.points], dtype=float)
    forces = np.array([point[1] for point in curve.points], dtype=float)
    impulse = cumulative_impulse(curve.points)
    total_impulse = impulse[-1]
    state = np.zeros(15)
    state[6] = 1.0
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
        if launched and state[1] < 0.0:
            break

    samples = decimate_samples(log, 500)
    apogee_sample = max(log, key=lambda sample: sample["altitude"])
    burnout_sample = min(log, key=lambda sample: abs(sample["time"] - curve.burn_time))
    return {
        "engine": "Pointy TVC",
        "motor": curve_summary(curve),
        "summary": {
            "apogeeM": apogee_sample["altitude"],
            "apogeeTimeS": apogee_sample["time"],
            "dryCgM": cg_dry,
            "launchCgM": launch_cg,
            "cpM": cp,
            "staticMarginCal": (cp - launch_cg) / (radius * 2),
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


def cumulative_impulse(points):
    impulse = [0.0]
    for index in range(1, len(points)):
        t0, f0 = points[index - 1]
        t1, f1 = points[index]
        impulse.append(impulse[-1] + 0.5 * (f0 + f1) * (t1 - t0))
    return impulse


def dry_cg(specs, airframe_mass, motor_dry_mass, length):
    components = specs.get("components")
    if not isinstance(components, list) or not components:
        return bounded(specs, "centerOfMass", 0.35, -length, length)
    total_mass = 0.0
    moment = 0.0
    for index, component in enumerate(components[:24]):
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
    import numpy as np

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
    import numpy as np

    position = state[:3]
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


def quaternion_multiply(first, second):
    import numpy as np

    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def conjugate(quaternion):
    import numpy as np

    return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])


def rotate_vector(quaternion, vector):
    import numpy as np

    q_vector = np.array([0.0, vector[0], vector[1], vector[2]])
    return quaternion_multiply(quaternion_multiply(quaternion, q_vector), conjugate(quaternion))[1:]


def normalize_quaternion(quaternion):
    import numpy as np

    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def curve_summary(curve):
    return {
        "name": curve.name,
        "burnTimeS": curve.burn_time,
        "totalImpulseNS": curve.total_impulse,
        "averageThrustN": curve.average_thrust,
        "peakThrustN": curve.peak_thrust,
        "points": [{"time": time_value, "thrust": force_value} for time_value, force_value in curve.points],
    }


def inferred_motor_dry_mass(curve):
    if curve.total_mass_kg and curve.propellant_mass_kg and curve.total_mass_kg > curve.propellant_mass_kg:
        return curve.total_mass_kg - curve.propellant_mass_kg
    return 0.0422


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


def parse_float(value, label):
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"Invalid {label}.") from error
    if not math.isfinite(parsed):
        raise ValueError(f"Invalid {label}.")
    return parsed


def optional_float(value, label):
    return parse_float(value, label) if value not in (None, "") else None


def optional_grams_to_kg(value):
    return optional_float(value, "mass") / 1000 if value not in (None, "") else None


def grams_to_kg(total_value, propellant_value):
    if propellant_value not in (None, ""):
        return optional_float(propellant_value, "propellant mass") / 1000
    return None
