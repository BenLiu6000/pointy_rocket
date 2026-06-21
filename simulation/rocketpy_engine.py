"""Trusted passive flight + aerodynamics via RocketPy.

RocketPy gives us Barrowman centre-of-pressure, a real atmosphere, rail-launch
dynamics and an industry-validated 6-DOF trajectory -- but it has no thrust
vectoring. So this module runs the *passive* (uncontrolled) flight and, more
importantly, hands its trusted aerodynamics back to the TVC sim so the steering
is simulated against believable forces.

Coordinate convention used here: ``tail_to_nose`` with the origin at the rocket
base (tail). Positions therefore increase toward the nose and match the
"distance from base" used everywhere else in this project (the .ork importer,
the component table and ``tvc_sim``).

If RocketPy is not installed the module imports cleanly and reports
``available=False`` so the rest of the app keeps working with the analytic
fallback.
"""

from __future__ import annotations

import math
import os

os.environ.setdefault("MPLBACKEND", "Agg")

try:
    from rocketpy import Environment, Flight, GenericMotor, Rocket

    ROCKETPY_AVAILABLE = True
    ROCKETPY_IMPORT_ERROR = None
except Exception as error:  # pragma: no cover - depends on environment
    ROCKETPY_AVAILABLE = False
    ROCKETPY_IMPORT_ERROR = str(error)

from tvc_sim import bounded, dry_cg, positive


NOSE_KINDS = {
    "vonkarman": "vonKarman",
    "ogive": "ogive",
    "cone": "conical",
    "conical": "conical",
    "elliptical": "elliptical",
    "lvhaack": "lvhaack",
}

# RocketPy's adaptive solver crawls (effectively hangs) on a statically unstable
# rocket because it integrates the tumble. A TVC vehicle is unstable by design,
# so we only run the passive RocketPy trajectory when the rocket is genuinely
# stable; otherwise we still return the (instant) Barrowman static aerodynamics
# and let the TVC sim provide the controlled flight.
STABLE_MARGIN_CAL = 0.5


def run_rocketpy_flight(curve, specs):
    """Run the passive RocketPy flight.

    Returns a dict with ``available`` plus, when successful, ``summary``,
    ``samples`` (altitude AGL vs time) and ``derived`` aerodynamics for the TVC
    sim. Never raises: failures come back as ``{"available": False, "error": ...}``
    so the caller can fall back to the analytic path.
    """
    if not ROCKETPY_AVAILABLE:
        return {"available": False, "error": ROCKETPY_IMPORT_ERROR or "RocketPy is not installed."}
    try:
        return _build_and_fly(curve, specs)
    except Exception as error:  # pragma: no cover - RocketPy internals vary
        return {"available": False, "error": f"RocketPy flight failed: {error}"}


def _build_and_fly(curve, specs):
    radius = positive(specs, "radius", 0.05)
    length = positive(specs, "length", 0.99)
    nose_length = positive(specs, "noseLength", 0.22)
    drag = bounded(specs, "dragCoefficient", 0.75, 0.05, 3.0)
    dry_mass_without_motor = positive(specs, "dryMass", 1.061)
    motor_dry_mass = positive(specs, "motorDryMass", curve.motor_dry_mass or 0.0422)
    propellant_mass = positive(specs, "motorPropellantMass", curve.propellant_mass_kg or 0.0941)
    motor_position = bounded(specs, "motorPosition", 0.0, -length, length)
    inertia_i = positive(specs, "inertiaI", 0.088)
    inertia_z = positive(specs, "inertiaZ", 0.0013)

    rail_length = bounded(specs, "railLength", 1.0, 0.1, 50.0)
    inclination = bounded(specs, "inclination", 85.0, 1.0, 90.0)
    heading = bounded(specs, "heading", 90.0, 0.0, 360.0)
    latitude = bounded(specs, "latitude", 53.3498, -90.0, 90.0)
    longitude = bounded(specs, "longitude", -6.2603, -180.0, 180.0)
    elevation = bounded(specs, "elevation", 20.0, -500.0, 9000.0)
    max_time = bounded(specs, "maxTime", 120.0, 1.0, 600.0)
    terminate_on_apogee = bool(specs.get("terminateOnApogee"))

    cg_dry = dry_cg(specs, dry_mass_without_motor, motor_dry_mass, length)

    environment = Environment(latitude=latitude, longitude=longitude, elevation=elevation)
    environment.set_atmospheric_model(type="standard_atmosphere")

    motor = _build_motor(curve, motor_dry_mass, propellant_mass)

    rocket = Rocket(
        radius=radius,
        mass=dry_mass_without_motor,
        inertia=(inertia_i, inertia_i, inertia_z),
        power_off_drag=drag,
        power_on_drag=drag,
        center_of_mass_without_motor=cg_dry,
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=motor_position)
    rocket.add_nose(
        length=nose_length,
        kind=NOSE_KINDS.get(str(specs.get("noseShape", "vonKarman")).lower(), "vonKarman"),
        position=length,
    )

    # Static (Barrowman) aerodynamics -- instant, and the trusted values we hand
    # to the TVC sim. RocketPy's static_margin already carries the correct sign
    # (negative => CP forward of CG => unstable).
    cp_from_base = _evaluate(rocket.cp_position, 0.0)
    launch_cg = _evaluate(rocket.center_of_mass, 0.0)
    static_margin = _evaluate(rocket.static_margin, 0.0)

    summary = {
        "dryCgM": cg_dry,
        "launchCgM": launch_cg,
        "cpM": cp_from_base,
        "staticMarginCal": static_margin,
    }
    warnings = []
    passive_samples = []
    passive_flight_ran = static_margin >= STABLE_MARGIN_CAL

    if passive_flight_ran:
        # Stable enough for RocketPy to fly. Cap steps/time so a borderline case
        # can never hang the request.
        flight = Flight(
            rocket=rocket,
            environment=environment,
            rail_length=rail_length,
            inclination=inclination,
            heading=heading,
            max_time=max_time,
            max_time_step=0.1,
            terminate_on_apogee=True if not terminate_on_apogee else terminate_on_apogee,
        )
        summary.update(
            {
                "apogeeM": float(flight.apogee) - elevation,
                "apogeeTimeS": float(flight.apogee_time),
                "offRailStabilityCal": float(flight.out_of_rail_stability_margin),
                "offRailSpeedMS": float(flight.out_of_rail_velocity),
                "maxSpeedMS": float(flight.max_speed),
                "maxSpeedTimeS": float(flight.max_speed_time),
                "flightTimeS": float(flight.t_final),
            }
        )
        passive_samples = _altitude_samples(flight, elevation)
    else:
        warnings.append(
            f"Statically unstable (margin {static_margin:.2f} cal): RocketPy cannot fly "
            "it passively, so the passive trajectory is skipped. This is expected for a "
            "TVC vehicle -- the in-house TVC sim provides the controlled flight."
        )

    return {
        "available": True,
        "passiveFlightRan": passive_flight_ran,
        "summary": summary,
        "samples": passive_samples,
        "derived": {"cpFromBase": cp_from_base, "drag": drag, "source": "RocketPy"},
        "warnings": warnings,
    }


def _build_motor(curve, motor_dry_mass, propellant_mass):
    chamber_radius = max(0.005, (curve.diameter_mm or 24.0) / 2 / 1000 * 0.9)
    chamber_height = max(0.02, (curve.length_mm or 70.0) / 1000 * 0.9)
    return GenericMotor(
        thrust_source=[[time_value, force] for time_value, force in curve.points],
        burn_time=curve.burn_time,
        chamber_radius=chamber_radius,
        chamber_height=chamber_height,
        chamber_position=chamber_height / 2,
        propellant_initial_mass=propellant_mass,
        nozzle_radius=chamber_radius * 0.5,
        dry_mass=motor_dry_mass,
        center_of_dry_mass_position=chamber_height / 2,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )


def _altitude_samples(flight, elevation, limit=200):
    source = getattr(flight.altitude, "source", None)
    points = []
    try:
        for row in source:
            points.append((float(row[0]), float(row[1])))
    except TypeError:
        points = []
    if not points:
        end = float(flight.t_final)
        steps = 120
        points = [(end * i / steps, float(flight.altitude(end * i / steps))) for i in range(steps + 1)]
    if len(points) > limit:
        step = max(1, len(points) // limit)
        decimated = points[::step]
        if decimated[-1] != points[-1]:
            decimated.append(points[-1])
        points = decimated
    return [{"time": time_value, "altitude": max(0.0, altitude)} for time_value, altitude in points]


def _evaluate(function_or_value, argument):
    """RocketPy exposes some properties as callables (Function) and some as floats."""
    try:
        return float(function_or_value(argument))
    except TypeError:
        return float(function_or_value)
