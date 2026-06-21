"""Matplotlib rendering of a combined simulation result.

The browser GUI draws a quick canvas chart, but several uses (reports, slides,
sharing a single image) want a proper matplotlib figure. This module turns the
``run_simulation`` payload into a 4-panel PNG:

  1. Altitude vs time   - TVC (active) with the RocketPy passive overlay if present
  2. Motor thrust curve
  3. TVC gimbal angles  - pitch and yaw vs time
  4. Speed and mass vs time

It is used two ways:
  * ``render_png(result)`` -> PNG bytes, attached to the GUI ``/api/simulate``
    response so the matplotlib figure shows next to the canvas;
  * ``python plots.py <motorfile>`` -> runs a default sim and saves
    ``matplotlib_results.png`` (mirrors the standalone ``rocket_sim.py``).
"""

from __future__ import annotations

import io
import os

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def render_png(result):
    figure = build_figure(result)
    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=110)
    plt.close(figure)
    return buffer.getvalue()


def build_figure(result):
    samples = result.get("samples", [])
    motor_points = result.get("motor", {}).get("points", [])
    rocketpy = result.get("rocketpy", {})
    rp_samples = rocketpy.get("samples", []) if rocketpy.get("available") else []
    summary = result.get("summary", {})

    times = [s["time"] for s in samples]

    figure, axes = plt.subplots(2, 2, figsize=(11, 7))
    figure.suptitle(_title(result, summary), fontsize=12, fontweight="bold")

    # 1. Altitude
    ax = axes[0][0]
    ax.plot(times, [s["altitude"] for s in samples], color="#5b513f", label="TVC (active)")
    if rp_samples:
        ax.plot([s["time"] for s in rp_samples], [s["altitude"] for s in rp_samples], color="#2f6b4f", linestyle="--", label="RocketPy (passive)")
    ax.set_title("Altitude")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. Thrust curve
    ax = axes[0][1]
    ax.plot([p["time"] for p in motor_points], [p["thrust"] for p in motor_points], color="#9a6b32")
    ax.set_title(f"Thrust curve - {result.get('motor', {}).get('name', 'motor')}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Thrust (N)")
    ax.grid(True, alpha=0.3)

    # 3. Gimbal angles
    ax = axes[1][0]
    ax.plot(times, [s["gimbalPitchDeg"] for s in samples], color="#b4452f", label="Pitch")
    ax.plot(times, [s["gimbalYawDeg"] for s in samples], color="#2f6bb4", label="Yaw")
    ax.set_title("TVC gimbal angle")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Speed and mass
    ax = axes[1][1]
    ax.plot(times, [s["speed"] for s in samples], color="#5b513f", label="Speed")
    ax.set_title("Speed and mass")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.3)
    twin = ax.twinx()
    twin.plot(times, [s["mass"] for s in samples], color="#9a6b32", label="Mass")
    twin.set_ylabel("Mass (kg)")
    lines = ax.get_lines() + twin.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], fontsize=8)

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    return figure


def _title(result, summary):
    parts = [result.get("engine", "Pointy Rocket")]
    if "apogeeM" in summary:
        parts.append(f"apogee {summary['apogeeM']:.0f} m")
    if "staticMarginCal" in summary:
        parts.append(f"static margin {summary['staticMarginCal']:.1f} cal")
    if "maxGimbalDeg" in summary:
        parts.append(f"max gimbal {summary['maxGimbalDeg']:.1f} deg")
    return "  |  ".join(parts)


def _demo(motor_path=None):
    from rocketpy_adapter import parse_motor_curve, run_simulation

    if motor_path:
        with open(motor_path, "r", encoding="utf-8", errors="replace") as handle:
            content = handle.read()
        file_name = os.path.basename(motor_path)
    else:
        file_name = "F35.eng"
        content = "F35 24 70 0 0.035 0.080 Demo\n0.05 20\n0.1 70\n0.3 55\n0.6 45\n1.0 38\n1.2 10\n1.3 0\n"
    curve = parse_motor_curve(file_name, content)
    specs = {
        "dryMass": 1.04, "radius": 0.05, "length": 0.9, "noseLength": 0.20,
        "motorPropellantMass": curve.propellant_mass_kg or 0.035,
        "motorDryMass": curve.motor_dry_mass or 0.045,
        "components": [
            {"name": "Airframe", "mass": 0.42, "position": 0.45},
            {"name": "Nose", "mass": 0.12, "position": 0.80},
            {"name": "Avionics", "mass": 0.30, "position": 0.55},
            {"name": "TVC mount", "mass": 0.20, "position": 0.08},
        ],
    }
    result = run_simulation(curve, specs)
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "matplotlib_results.png")
    with open(output, "wb") as handle:
        handle.write(render_png(result))
    print(f"Saved {output}  ({result['engine']}, apogee {result['summary']['apogeeM']:.1f} m)")


if __name__ == "__main__":
    import sys

    _demo(sys.argv[1] if len(sys.argv) > 1 else None)
