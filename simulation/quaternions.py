"""Quaternion helpers shared by the physics sim and the flight computer.

Convention: a quaternion ``q = [w, x, y, z]`` rotates a vector from the BODY
frame to the WORLD frame (``rotate_vector(q, v_body) -> v_world``). The world
frame has +Y up. Kept in its own module so ``tvc_sim`` and ``flight_computer``
can both use it without a circular import.
"""

from __future__ import annotations

import math

import numpy as np


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
    """Rotate ``vector`` from body to world frame."""
    q_vector = np.array([0.0, vector[0], vector[1], vector[2]])
    return quaternion_multiply(quaternion_multiply(quaternion, q_vector), conjugate(quaternion))[1:]


def normalize_quaternion(quaternion):
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quaternion_to_matrix(quaternion):
    """3x3 rotation matrix R such that ``R @ v_body = v_world`` (q must be normalised).

    Building the matrix once and reusing it for several rotations is much cheaper
    than calling ``rotate_vector`` repeatedly in the integration hot loop.
    """
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_rotvec(rotvec):
    """Exact rotation quaternion for a rotation vector (axis * angle, radians).

    Used to integrate a (locally constant) body rate over a time step without the
    small-angle error of the linear ``q += 0.5 q*omega dt`` update.
    """
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = np.asarray(rotvec) / angle
    half = angle / 2.0
    sin_half = math.sin(half)
    return np.array([math.cos(half), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half])


def quaternion_between(vector_from, vector_to):
    """Shortest-arc body->world quaternion mapping ``vector_from`` onto ``vector_to``.

    Both vectors are normalised internally. Handles the parallel and anti-parallel
    degenerate cases. Used to initialise attitude from a measured gravity vector.
    """
    a = np.asarray(vector_from, dtype=float)
    b = np.asarray(vector_to, dtype=float)
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm < 1e-12 or b_norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    a = a / a_norm
    b = b / b_norm
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -1.0 + 1e-9:
        # 180 deg: pick any axis perpendicular to a.
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 0.0, 1.0]))
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    axis = np.cross(a, b)
    return normalize_quaternion(np.array([1.0 + dot, axis[0], axis[1], axis[2]]))


def launch_quaternion(inclination_deg, heading_deg):
    """Body->world quaternion for a rocket sitting on the rail.

    ``inclination`` is measured from horizontal (90 deg = vertical), matching
    RocketPy's Flight convention, so both engines leave the pad at the same angle.
    Tilts the body +Y (longitudinal) axis to the launch direction.
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
    return quaternion_between(np.array([0.0, 1.0, 0.0]), direction)


def tilt_from_vertical_deg(quaternion):
    """Angle (degrees) between the body +Y axis and world up, given body->world q."""
    body_axis_world = rotate_vector(quaternion, np.array([0.0, 1.0, 0.0]))
    cos_tilt = float(np.clip(body_axis_world[1], -1.0, 1.0))
    return math.degrees(math.acos(cos_tilt))
