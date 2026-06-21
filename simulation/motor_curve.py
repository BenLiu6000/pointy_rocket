"""Motor thrust-curve parsing shared by both flight engines.

Reads RASP ``.eng``, RockSim ``.rse`` and plain ``.csv`` thrust curves
(e.g. downloaded from thrustcurve.org) into a normalised ``MotorCurve``.
The same curve is consumed by the RocketPy engine (``rocketpy_engine.py``)
and the in-house TVC sim (``tvc_sim.py``).
"""

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

    @property
    def motor_dry_mass(self):
        """Casing/hardware mass inferred from the .eng/.rse header, if present."""
        if self.total_mass_kg and self.propellant_mass_kg and self.total_mass_kg > self.propellant_mass_kg:
            return self.total_mass_kg - self.propellant_mass_kg
        return None


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


def cumulative_impulse(points):
    impulse = [0.0]
    for index in range(1, len(points)):
        t0, f0 = points[index - 1]
        t1, f1 = points[index]
        impulse.append(impulse[-1] + 0.5 * (f0 + f1) * (t1 - t0))
    return impulse


def curve_summary(curve):
    return {
        "name": curve.name,
        "burnTimeS": curve.burn_time,
        "totalImpulseNS": curve.total_impulse,
        "averageThrustN": curve.average_thrust,
        "peakThrustN": curve.peak_thrust,
        "points": [{"time": time_value, "thrust": force_value} for time_value, force_value in curve.points],
    }


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
