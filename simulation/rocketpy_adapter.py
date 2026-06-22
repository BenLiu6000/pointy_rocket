"""Orchestrator that combines the two flight engines.

The web GUI talks only to this module. It:

1. parses the uploaded motor curve (``motor_curve``),
2. runs RocketPy for trusted Barrowman aerodynamics + (when the rocket is stable)
   a passive trajectory (``rocketpy_engine``),
3. feeds RocketPy's centre of pressure / drag into the in-house TVC sim so the
   active, steered flight is simulated against believable forces (``tvc_sim``),
4. returns a single payload carrying both the active (TVC) result and the
   passive (RocketPy) result.

``parse_motor_curve`` and ``run_simulation`` are re-exported here so existing
imports in ``gui_server`` keep working.
"""

from __future__ import annotations

from importlib import metadata

from motor_curve import curve_summary, parse_motor_curve  # noqa: F401 (re-exported)
from rocketpy_engine import ROCKETPY_AVAILABLE, run_rocketpy_flight
from tvc_sim import run_tvc_flight


def run_simulation(curve, specs):
    rocketpy_result = run_rocketpy_flight(curve, specs)
    aero = rocketpy_result.get("derived") if rocketpy_result.get("available") else None
    tvc_result = run_tvc_flight(curve, specs, aero=aero)
    return {
        "engine": _engine_label(rocketpy_result, tvc_result),
        "aeroSource": tvc_result["aeroSource"],
        "motor": curve_summary(curve),
        "summary": tvc_result["summary"],
        "samples": tvc_result["samples"],
        "rocketpy": rocketpy_result,
    }


def _engine_label(rocketpy_result, tvc_result):
    if rocketpy_result.get("available"):
        passive = "passive flight + " if rocketpy_result.get("passiveFlightRan") else ""
        return f"RocketPy {_rocketpy_version()} ({passive}aero) + Pointy TVC"
    return "Pointy TVC (analytic aero; RocketPy unavailable)"


def _rocketpy_version():
    try:
        return metadata.version("rocketpy")
    except metadata.PackageNotFoundError:
        return "?"
