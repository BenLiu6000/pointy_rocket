"""Software-in-the-loop model of the Pointy Rocket flight computer.

This is the realistic GNC chain that the in-flight sim drives instead of giving
the controller perfect truth:

    true motion -> MPU6050 (noise/bias/quantisation) -> attitude estimator
                -> discrete PID -> servo (slew/resolution/lag) -> gimbal

Hardware modelled from the actual board/firmware (``Firmware_test/src/main.cpp``):
MPU6050 read with ``getMotion6`` at its defaults (+/-2 g, +/-250 deg/s, 16-bit),
and two hobby servos driven by the Arduino ``Servo`` library (1 deg write
resolution) on the TVC gimbal.

Key physics constraint that drives the estimator design: in flight a rocket's
accelerometer does NOT see gravity (under thrust it reads thrust along the body
axis; in coast it is in free fall and reads ~0). So attitude is propagated by
integrating the gyro, with the accelerometer used only on the pad to set the
initial attitude and to calibrate the gyro bias. Residual gyro bias therefore
drives attitude drift - the dominant real-world error, which this model exposes.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from quaternions import (
    conjugate,
    normalize_quaternion,
    quaternion_between,
    quaternion_from_rotvec,
    quaternion_multiply,
    rotate_vector,
)

GRAVITY = 9.80665


@dataclass
class FlightComputerParams:
    # Loop timing
    control_rate_hz: float = 100.0
    loop_latency_s: float = 0.006          # sense -> compute -> actuate transport delay
    pad_duration_s: float = 2.0            # time on the rail used to calibrate

    # MPU6050 (datasheet-derived; defaults match the firmware's initialize())
    gyro_range_dps: float = 250.0
    gyro_lsb_per_dps: float = 131.0
    gyro_noise_dps: float = 0.05           # white noise RMS
    gyro_bias_sigma_dps: float = 3.0       # per-flight constant bias (uncalibrated chip)
    accel_range_g: float = 2.0
    accel_lsb_per_g: float = 16384.0
    accel_noise_g: float = 0.008
    accel_bias_sigma_g: float = 0.02

    # Controller (gains come from the GUI specs; mirrored onto the Teensy)
    kp: float = 0.8
    kd: float = 1.5
    ki: float = 0.0
    max_gimbal_rad: float = math.radians(8.0)
    integral_limit: float = 0.5
    # Feedback signs per axis (proportional/integral share a sign; derivative its own).
    # Determined empirically (sign sweep) so the loop drives tilt toward vertical
    # during powered flight. The damping sign is asymmetric between the pitch and
    # yaw channels because the body-rate axes map oppositely to the two tilt
    # directions through the gimbal/cross-product geometry.
    sign_prop_pitch: float = -1.0
    sign_der_pitch: float = -1.0
    sign_prop_yaw: float = -1.0
    sign_der_yaw: float = 1.0

    # Servo + linkage (gimbal-referred: depends on servo speed and the linkage ratio)
    servo_slew_dps: float = 400.0          # max gimbal rate
    servo_resolution_deg: float = 0.5      # gimbal command quantisation (Servo.write is 1 deg at the horn)
    servo_tau_s: float = 0.02              # first-order lag
    servo_deadband_deg: float = 0.0


class MPU6050:
    """MPU6050 measurement model: per-flight bias + white noise + range clip + LSB quantisation."""

    def __init__(self, rng, params):
        self.rng = rng
        self.gyro_range = math.radians(params.gyro_range_dps)
        self.gyro_lsb_per_rad = params.gyro_lsb_per_dps / math.radians(1.0)
        self.gyro_noise = math.radians(params.gyro_noise_dps)
        self.accel_range = params.accel_range_g * GRAVITY
        self.accel_lsb_per_ms2 = params.accel_lsb_per_g / GRAVITY
        self.accel_noise = params.accel_noise_g * GRAVITY
        # Constant biases drawn once per flight.
        self.gyro_bias = rng.normal(0.0, math.radians(params.gyro_bias_sigma_dps), 3)
        self.accel_bias = rng.normal(0.0, params.accel_bias_sigma_g * GRAVITY, 3)

    @staticmethod
    def _quantize(value, lsb_per_unit, limit):
        clipped = np.clip(value, -limit, limit)
        return np.round(clipped * lsb_per_unit) / lsb_per_unit

    def read_gyro(self, true_rate_body):
        raw = true_rate_body + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise, 3)
        return self._quantize(raw, self.gyro_lsb_per_rad, self.gyro_range)

    def read_accel(self, specific_force_body):
        raw = specific_force_body + self.accel_bias + self.rng.normal(0.0, self.accel_noise, 3)
        return self._quantize(raw, self.accel_lsb_per_ms2, self.accel_range)


class AttitudeEstimator:
    """Gyro-integration attitude estimator with on-pad calibration.

    On the pad the accelerometer sees gravity, so it initialises attitude and the
    averaged gyro gives the bias estimate. In flight the quaternion is propagated
    purely from the bias-corrected gyro (the accelerometer is not a valid gravity
    reference once airborne).
    """

    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.bias = np.zeros(3)
        self._gyro_sum = np.zeros(3)
        self._accel_sum = np.zeros(3)
        self._samples = 0
        self.calibrated = False

    def calibrate_sample(self, gyro, accel):
        self._gyro_sum += gyro
        self._accel_sum += accel
        self._samples += 1

    def finalize_calibration(self):
        if self._samples > 0:
            self.bias = self._gyro_sum / self._samples
            mean_accel = self._accel_sum / self._samples
            # mean_accel is the measured gravity-up direction in the body frame;
            # rotate it to world +Y to recover the body->world attitude.
            self.q = quaternion_between(mean_accel, np.array([0.0, 1.0, 0.0]))
        self.calibrated = True

    def update(self, gyro, accel, dt):
        rate = gyro - self.bias
        self.q = normalize_quaternion(quaternion_multiply(self.q, quaternion_from_rotvec(rate * dt)))
        return self.q, rate


class TVCController:
    """Discrete dual-axis PID on the estimated attitude, with integral clamping."""

    def __init__(self, params):
        self.kp = params.kp
        self.kd = params.kd
        self.ki = params.ki
        self.max_gimbal = params.max_gimbal_rad
        self.integral_limit = params.integral_limit
        self.sign_prop_pitch = params.sign_prop_pitch
        self.sign_der_pitch = params.sign_der_pitch
        self.sign_prop_yaw = params.sign_prop_yaw
        self.sign_der_yaw = params.sign_der_yaw
        self.integral = np.zeros(3)

    def update(self, q_hat, rate_est, dt):
        # World-up expressed in the (estimated) body frame; its lateral components
        # are the tilt error the gimbal must null.
        up_body = rotate_vector(conjugate(q_hat), np.array([0.0, 1.0, 0.0]))
        error = np.array([up_body[0], 0.0, up_body[2]])
        self.integral = np.clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        target_pitch = (
            self.sign_prop_pitch * (self.kp * error[2] + self.ki * self.integral[2])
            + self.sign_der_pitch * self.kd * rate_est[0]
        )
        target_yaw = (
            self.sign_prop_yaw * (self.kp * error[0] + self.ki * self.integral[0])
            + self.sign_der_yaw * self.kd * rate_est[2]
        )
        target_pitch = float(np.clip(target_pitch, -self.max_gimbal, self.max_gimbal))
        target_yaw = float(np.clip(target_yaw, -self.max_gimbal, self.max_gimbal))
        return np.array([target_pitch, target_yaw]), error


class Servo:
    """Single gimbal axis: command quantisation + deadband, first-order lag, slew-rate limit."""

    def __init__(self, params):
        self.slew = math.radians(params.servo_slew_dps)
        self.resolution = math.radians(params.servo_resolution_deg)
        self.tau = params.servo_tau_s
        self.deadband = math.radians(params.servo_deadband_deg)
        self.position = 0.0
        self._command = 0.0

    def set_command(self, target):
        if self.resolution > 0:
            target = round(target / self.resolution) * self.resolution
        if abs(target - self._command) >= self.deadband:
            self._command = target

    def step(self, dt):
        if self.tau > 1e-9:
            desired = self.position + (self._command - self.position) * (1.0 - math.exp(-dt / self.tau))
        else:
            desired = self._command
        delta = desired - self.position
        max_delta = self.slew * dt
        self.position += max(-max_delta, min(max_delta, delta))
        return self.position


class FlightComputer:
    """Ties the chain together at the control rate, with transport latency."""

    def __init__(self, params, rng):
        self.params = params
        self.sensor = MPU6050(rng, params)
        self.estimator = AttitudeEstimator()
        self.controller = TVCController(params)
        self.servo_pitch = Servo(params)
        self.servo_yaw = Servo(params)
        self.control_dt = 1.0 / params.control_rate_hz
        self.latency_steps = max(0, int(round(params.loop_latency_s / self.control_dt)))
        self._buffer = deque([np.zeros(2) for _ in range(self.latency_steps)])
        self.command = np.zeros(2)
        # diagnostics exposed for logging
        self.q_hat = np.array([1.0, 0.0, 0.0, 0.0])
        self.rate_est = np.zeros(3)
        self.error = np.zeros(3)

    def calibrate(self, true_quat):
        """Run the on-pad calibration: the rocket sits on the rail reading gravity."""
        samples = max(1, int(round(self.params.pad_duration_s * self.params.control_rate_hz)))
        specific_force_body = rotate_vector(conjugate(true_quat), np.array([0.0, GRAVITY, 0.0]))
        for _ in range(samples):
            gyro = self.sensor.read_gyro(np.zeros(3))
            accel = self.sensor.read_accel(specific_force_body)
            self.estimator.calibrate_sample(gyro, accel)
        self.estimator.finalize_calibration()
        self.q_hat = self.estimator.q.copy()

    def control_tick(self, true_rate_body, specific_force_body):
        gyro = self.sensor.read_gyro(true_rate_body)
        accel = self.sensor.read_accel(specific_force_body)
        self.q_hat, self.rate_est = self.estimator.update(gyro, accel, self.control_dt)
        target, self.error = self.controller.update(self.q_hat, self.rate_est, self.control_dt)
        if self.latency_steps > 0:
            self._buffer.append(target)
            self.command = self._buffer.popleft()
        else:
            self.command = target
        self.servo_pitch.set_command(self.command[0])
        self.servo_yaw.set_command(self.command[1])

    def actuate(self, dt):
        return np.array([self.servo_pitch.step(dt), self.servo_yaw.step(dt)])

    @property
    def gyro_bias_dps(self):
        return float(np.degrees(np.linalg.norm(self.sensor.gyro_bias)))

    @property
    def bias_residual_dps(self):
        """Bias left after on-pad calibration - this is what drives in-flight drift."""
        return float(np.degrees(np.linalg.norm(self.sensor.gyro_bias - self.estimator.bias)))


def params_from_specs(specs, bounded):
    """Build FlightComputerParams from GUI specs using the sim's bounded() validator."""
    return FlightComputerParams(
        control_rate_hz=bounded(specs, "controlRateHz", 100.0, 10.0, 1000.0),
        loop_latency_s=bounded(specs, "loopLatencyS", 0.006, 0.0, 0.2),
        pad_duration_s=bounded(specs, "padDurationS", 2.0, 0.1, 30.0),
        gyro_range_dps=bounded(specs, "gyroRangeDps", 250.0, 125.0, 2000.0),
        gyro_noise_dps=bounded(specs, "gyroNoiseDps", 0.05, 0.0, 10.0),
        gyro_bias_sigma_dps=bounded(specs, "gyroBiasSigmaDps", 3.0, 0.0, 30.0),
        accel_noise_g=bounded(specs, "accelNoiseG", 0.008, 0.0, 1.0),
        accel_bias_sigma_g=bounded(specs, "accelBiasSigmaG", 0.02, 0.0, 1.0),
        kp=bounded(specs, "kp", 0.8, 0.0, 100.0),
        kd=bounded(specs, "kd", 1.5, 0.0, 100.0),
        ki=bounded(specs, "ki", 0.0, 0.0, 100.0),
        max_gimbal_rad=math.radians(bounded(specs, "maxGimbalDeg", 8.0, 0.0, 25.0)),
        sign_prop_pitch=bounded(specs, "signPropPitch", 1.0, -1.0, 1.0),
        sign_der_pitch=bounded(specs, "signDerPitch", -1.0, -1.0, 1.0),
        sign_prop_yaw=bounded(specs, "signPropYaw", 1.0, -1.0, 1.0),
        sign_der_yaw=bounded(specs, "signDerYaw", 1.0, -1.0, 1.0),
        servo_slew_dps=bounded(specs, "servoSlewDps", 400.0, 1.0, 5000.0),
        servo_resolution_deg=bounded(specs, "servoResolutionDeg", 0.5, 0.0, 5.0),
        servo_tau_s=bounded(specs, "servoTau", 0.02, 0.0, 1.0),
        servo_deadband_deg=bounded(specs, "servoDeadbandDeg", 0.0, 0.0, 5.0),
    )
