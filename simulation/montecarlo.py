"""Monte-Carlo flight dispersion for the TVC rocket.

Like a NASA dispersion analysis: run many flights, each with the uncertain
parameters perturbed (mass, propellant, motor thrust, drag, centre of pressure,
wind, thrust misalignment, and per-flight sensor bias/noise), then report the
distribution of outcomes and how often the vehicle stays controlled.

RocketPy's (trusted) aerodynamics are evaluated ONCE for the nominal vehicle;
each run then perturbs around that and calls the fast in-house ``run_tvc_flight``
directly, so RocketPy is not re-run per sample. Flights run in parallel across
CPU cores.

CLI:  python montecarlo.py <motor.eng> [runs] [ork_file]
"""

from __future__ import annotations

import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from motor_curve import MotorCurve, parse_motor_curve
from rocketpy_engine import run_rocketpy_flight
from tvc_sim import run_tvc_flight


@dataclass
class DispersionConfig:
    mass_cv: float = 0.03            # dry-mass coefficient of variation
    propellant_cv: float = 0.02
    thrust_cv: float = 0.05          # motor-to-motor total-thrust scatter
    drag_cv: float = 0.15
    cp_sigma_m: float = 0.02         # centre-of-pressure uncertainty (m)
    wind_sigma_ms: float = 1.5       # gust scatter added to nominal wind, per axis
    misalign_sigma_deg: float = 0.5  # thrust-axis misalignment
    # "stayed controlled" = max tilt DURING POWERED FLIGHT below this. (After
    # burnout a finless TVC rocket has no authority and tumbles - that is expected
    # and is not counted as a control failure.)
    tilt_success_deg: float = 20.0


def run_dispersion(curve, base_specs, runs=100, seed=0, config=None, workers=None):
    config = config or DispersionConfig()
    rocketpy = run_rocketpy_flight(curve, base_specs)
    if rocketpy.get("available") and rocketpy.get("derived"):
        base_cp = float(rocketpy["derived"]["cpFromBase"])
        base_drag = float(rocketpy["derived"]["drag"])
        aero_source = "RocketPy"
    else:
        base_cp = base_drag = None
        aero_source = "analytic"

    base_dry = float(base_specs.get("dryMass", 1.061))
    base_prop = float(base_specs.get("motorPropellantMass", curve.propellant_mass_kg or 0.0941))
    base_drag_spec = float(base_specs.get("dragCoefficient", 0.75))
    base_wind = (float(base_specs.get("windX", 0.0)), float(base_specs.get("windZ", 0.0)))

    master = np.random.default_rng(seed)
    jobs = [
        _make_job(curve, base_specs, config, base_cp, base_drag, base_dry, base_prop, base_drag_spec, base_wind,
                  np.random.default_rng(int(master.integers(0, 2**63 - 1))))
        for _ in range(runs)
    ]

    summaries = _execute(jobs, workers)
    return _aggregate(summaries, runs, aero_source, config)


def _make_job(curve, base_specs, config, base_cp, base_drag, base_dry, base_prop, base_drag_spec, base_wind, rng):
    specs = dict(base_specs)
    specs["dryMass"] = max(0.01, base_dry * rng.normal(1.0, config.mass_cv))
    specs["motorPropellantMass"] = max(1e-4, base_prop * rng.normal(1.0, config.propellant_cv))
    specs["windX"] = base_wind[0] + rng.normal(0.0, config.wind_sigma_ms)
    specs["windZ"] = base_wind[1] + rng.normal(0.0, config.wind_sigma_ms)
    specs["thrustMisalignPitchRad"] = float(rng.normal(0.0, math.radians(config.misalign_sigma_deg)))
    specs["thrustMisalignYawRad"] = float(rng.normal(0.0, math.radians(config.misalign_sigma_deg)))
    specs["seed"] = int(rng.integers(0, 2**31 - 1))  # drives this flight's sensor bias/noise
    thrust_scale = max(0.1, rng.normal(1.0, config.thrust_cv))
    run_curve = _scaled_curve(curve, thrust_scale)
    if base_cp is not None:
        aero = {
            "cpFromBase": base_cp + rng.normal(0.0, config.cp_sigma_m),
            "drag": max(0.05, base_drag * rng.normal(1.0, config.drag_cv)),
            "source": "RocketPy",
        }
    else:
        aero = None
        specs["dragCoefficient"] = max(0.05, base_drag_spec * rng.normal(1.0, config.drag_cv))
    return run_curve, specs, aero


def _scaled_curve(curve, scale):
    return MotorCurve(
        name=curve.name,
        points=[(time_value, force * scale) for time_value, force in curve.points],
        diameter_mm=curve.diameter_mm,
        length_mm=curve.length_mm,
        propellant_mass_kg=curve.propellant_mass_kg,
        total_mass_kg=curve.total_mass_kg,
    )


def _run_one(job):
    run_curve, specs, aero = job
    try:
        return run_tvc_flight(run_curve, specs, aero=aero)["summary"]
    except Exception as error:  # pragma: no cover - a bad draw shouldn't kill the batch
        return {"error": str(error)}


def _multiprocessing_safe():
    """Spawn-based multiprocessing needs __main__ to be a real importable file.

    When this module is driven from a heredoc/REPL/notebook, workers cannot
    re-import __main__ (it is ``<stdin>``), so fall back to serial instead.
    """
    import __main__

    path = getattr(__main__, "__file__", None)
    return bool(path) and os.path.isfile(path)


def _execute(jobs, workers):
    if workers is None:
        workers = max(1, min(8, (os.cpu_count() or 2) - 1))
    if workers <= 1 or not _multiprocessing_safe():
        return [_run_one(job) for job in jobs]
    try:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            return list(pool.map(_run_one, jobs))
    except Exception:
        return [_run_one(job) for job in jobs]


def _aggregate(summaries, runs, aero_source, config):
    ok = [s for s in summaries if "error" not in s]
    if not ok:
        return {"runs": runs, "completed": 0, "aeroSource": aero_source, "error": "all runs failed"}
    apogee = np.array([s["apogeeM"] for s in ok])
    powered_tilt = np.array([s["maxTiltPoweredDeg"] for s in ok])
    burnout_tilt = np.array([s["tiltAtBurnoutDeg"] for s in ok])
    gimbal = np.array([s["maxGimbalDeg"] for s in ok])
    speed = np.array([s["maxSpeedMS"] for s in ok])
    landing = np.array([math.hypot(s["finalX"], s["finalY"]) for s in ok])
    bias_residual = np.array([s.get("biasResidualDps", float("nan")) for s in ok])
    stable_mask = powered_tilt < config.tilt_success_deg
    stable_apogee = apogee[stable_mask]

    def stats(values):
        return {
            "mean": float(np.mean(values)),
            "std": float(np.std(values)),
            "p5": float(np.percentile(values, 5)),
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
        }

    return {
        "runs": runs,
        "completed": len(ok),
        "aeroSource": aero_source,
        "tiltThresholdDeg": config.tilt_success_deg,
        "stabilityRate": float(np.mean(stable_mask)),
        "gimbalSaturationRate": float(np.mean(gimbal >= gimbal.max() - 1e-6)) if gimbal.size else 0.0,
        "apogeeM": stats(apogee),
        "stableApogeeM": stats(stable_apogee) if stable_apogee.size else None,
        "poweredTiltDeg": {"p50": float(np.percentile(powered_tilt, 50)), "p95": float(np.percentile(powered_tilt, 95)), "max": float(powered_tilt.max())},
        "burnoutTiltDeg": {"p50": float(np.percentile(burnout_tilt, 50)), "p95": float(np.percentile(burnout_tilt, 95)), "max": float(burnout_tilt.max())},
        "maxGimbalDeg": {"p50": float(np.percentile(gimbal, 50)), "p95": float(np.percentile(gimbal, 95)), "max": float(gimbal.max())},
        "maxSpeedMS": stats(speed),
        "landingRadiusM": {"p50": float(np.percentile(landing, 50)), "p95": float(np.percentile(landing, 95)), "max": float(landing.max())},
        "biasResidualDps": {"p50": float(np.nanpercentile(bias_residual, 50)), "p95": float(np.nanpercentile(bias_residual, 95))},
    }


def format_report(result):
    if result.get("completed", 0) == 0:
        return f"Monte Carlo: 0/{result['runs']} runs completed ({result.get('error')})."
    lines = [
        f"Monte-Carlo dispersion: {result['completed']}/{result['runs']} flights  (aero: {result['aeroSource']})",
        f"  Powered-flight control: {result['stabilityRate'] * 100:.0f}% stayed within {result['tiltThresholdDeg']:.0f} deg of vertical during burn",
        f"  Gimbal saturation reached in {result['gimbalSaturationRate'] * 100:.0f}% of flights",
        "  Apogee (all):    mean {mean:.0f} m,  5th {p5:.0f},  50th {p50:.0f},  95th {p95:.0f}".format(**result["apogeeM"]),
    ]
    if result.get("stableApogeeM"):
        lines.append("  Apogee (controlled): mean {mean:.0f} m,  5th {p5:.0f},  95th {p95:.0f}".format(**result["stableApogeeM"]))
    lines += [
        "  Tilt during burn:    median {p50:.1f} deg,  95th {p95:.1f},  worst {max:.1f}".format(**result["poweredTiltDeg"]),
        "  Tilt at burnout:     median {p50:.1f} deg,  95th {p95:.1f},  worst {max:.1f}".format(**result["burnoutTiltDeg"]),
        "  Max gimbal:          median {p50:.1f} deg,  95th {p95:.1f},  worst {max:.1f}".format(**result["maxGimbalDeg"]),
        "  Landing radius from pad: median {p50:.0f} m,  95th {p95:.0f}  (post-burnout tumble + descent)".format(**result["landingRadiusM"]),
        "  Gyro-bias residual after calibration: median {p50:.3f} dps,  95th {p95:.3f}".format(**result["biasResidualDps"]),
    ]
    return "\n".join(lines)


def _default_specs(curve, components=None):
    specs = {
        "dryMass": 0.53, "radius": 0.0383, "length": 0.865, "noseLength": 0.18, "noseShape": "ogive",
        "dragCoefficient": 0.75, "motorPropellantMass": curve.propellant_mass_kg or 0.025,
        "motorDryMass": curve.motor_dry_mass or 0.06, "motorPosition": 0.0,
        "inertiaI": 0.05, "inertiaZ": 0.0004, "maxGimbalDeg": 8.0, "kp": 0.8, "kd": 1.5, "ki": 0.0,
        "inclination": 85.0, "heading": 90.0, "maxTime": 40.0,
    }
    if components:
        specs["components"] = components
    return specs


def main():
    if len(sys.argv) < 2:
        print("usage: python montecarlo.py <motor.eng> [runs] [ork_file]")
        return
    motor_path = sys.argv[1]
    runs = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    components = None
    if len(sys.argv) > 3:
        import base64
        from openrocket_import import import_rocket_body

        raw = Path(sys.argv[3]).read_bytes()
        body = import_rocket_body(Path(sys.argv[3]).name, base64.b64encode(raw).decode())
        components = body["components"]
    content = Path(motor_path).read_text(encoding="utf-8", errors="replace")
    curve = parse_motor_curve(Path(motor_path).name, content)
    specs = _default_specs(curve, components)
    print(f"Running {runs} dispersion flights for {curve.name}...")
    result = run_dispersion(curve, specs, runs=runs)
    print(format_report(result))


if __name__ == "__main__":
    main()
