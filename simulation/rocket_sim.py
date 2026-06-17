import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

# 1. Physical Parameters & 3D Definitions

initial_total_mass = 0.8  # kg
gravity_val = 9.802
gravity_vec = np.array([0.0, -gravity_val, 0.0])  # World Frame (Y is UP)

# Dynamic Mass Parameters
initial_propellant_mass = 0.0941  # kg
dry_mass = initial_total_mass - initial_propellant_mass

# Center of Gravity Parameters
cg_prop_from_base = 0.05  # m
cg_dry_from_base = 0.37597  # m

# Thrust Profile (Estes D12 x 4)
# High-fidelity data based on verified D12 RASP curve (values scaled x4)
thrust_curve_time = np.array([0.0, 0.05, 0.18, 0.28, 0.35, 0.88, 1.44, 1.61, 1.65])
thrust_curve_force = np.array([0.0, 10.3, 69.1, 118.9, 56.5, 36.4, 33.2, 18.5, 0.0])
burn_time = 1.65

# Calculate Cumulative Impulse
cum_impulse = np.zeros_like(thrust_curve_force)
for i in range(1, len(thrust_curve_time)):
    dt_step = thrust_curve_time[i] - thrust_curve_time[i-1]
    avg_f = 0.5 * (thrust_curve_force[i] + thrust_curve_force[i-1])
    cum_impulse[i] = cum_impulse[i-1] + avg_f * dt_step
total_impulse = cum_impulse[-1]

# Aerodynamic & Physical parameters
area_base = 0.00785  
Cd_base = 0.75
rho = 1.225  
height = 0.457  
radius = 0.05  # m (diameter 0.1)

# Parachute specs
parachute_area = 1 
parachute_cd = 1.5

# Fixed Aerodynamic Center of Pressure
cp_from_base = 0.60  # m

# Control Parameters
MAX_GIMBAL_ANGLE_DEG = 8.0
MAX_GIMBAL_ANGLE_RAD = math.radians(MAX_GIMBAL_ANGLE_DEG)
SERVO_TAU = 0.05  

Kp_gain = 0.8
Kd_gain = 1.5
Ki_gain = 0

# Wind
surface_wind = np.array([2.0, 0.0, 1.5])  # 3D Wind vector (m/s)
wind_shear_gradient = 0.05 


# 2. Quaternion & 3D Math Helpers

def q_mult(q1, q2):
    """Quaternion multiplication."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

def q_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_rotate_vec(q, v):
    """Rotates a vector v by quaternion q (Body to World)."""
    q_v = np.array([0, v[0], v[1], v[2]])
    return q_mult(q_mult(q, q_v), q_conjugate(q))[1:]

def q_normalize(q):
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([1.0, 0, 0, 0])


# 3. Physics & State Calculation

def get_physics_properties(t):
    if t >= thrust_curve_time[-1]:
        thrust = 0.0
        current_impulse = total_impulse
    else:
        thrust = np.interp(t, thrust_curve_time, thrust_curve_force)
        current_impulse = np.interp(t, thrust_curve_time, cum_impulse)
        
    prop_mass = initial_propellant_mass * max(0.0, 1.0 - (current_impulse / total_impulse))
    m = dry_mass + prop_mass
    
    cg = ((dry_mass * cg_dry_from_base) + (prop_mass * cg_prop_from_base)) / m
    
    # Inertia Tensor (Diagonal)
    # Body-Y is longitudinal axis
    Iyy = 0.5 * m * (radius**2)  # Roll
    Ixx = Izz = (1/12) * m * (height**2) + (1/4) * m * (radius**2) # Pitch/Yaw
    I_tensor = np.diag([Ixx, Iyy, Izz])
    
    return thrust, m, cg, I_tensor

def get_derivatives(t, state, est_error_vec, est_rate_vec, est_int_vec):
    """
    state = [pos(3), vel(3), quat(4), rates(3), gimbal(2)]
    pos: world
    vel: world
    quat: body to world
    rates: body (Roll, Pitch, Yaw)
    gimbal: [pitch_g, yaw_g]
    """
    pos = state[0:3]
    vel = state[3:6]
    quat = q_normalize(state[6:10])
    rates = state[10:13]
    gimbal = state[13:15]
    
    thrust_mag, mass, cg, I_tensor = get_physics_properties(t)
    
    # 1. Control Logic (2-Axis TVC)
    # v_up_body contains the World-UP vector in the Body Frame [X, Y, Z]
    # Pitch Gimbal (gp) -> Torques around X axis (fixes Z error)
    target_gp = - (Kp_gain * est_error_vec[2] + Kd_gain * est_rate_vec[0] + Ki_gain * est_int_vec[2])
    # Yaw Gimbal (gy) -> Torques around Z axis (fixes X error)
    target_gy = (Kp_gain * est_error_vec[0] + Kd_gain * est_rate_vec[2] + Ki_gain * est_int_vec[0])
    
    target_gp = np.clip(target_gp, -MAX_GIMBAL_ANGLE_RAD, MAX_GIMBAL_ANGLE_RAD)
    target_gy = np.clip(target_gy, -MAX_GIMBAL_ANGLE_RAD, MAX_GIMBAL_ANGLE_RAD)
    
    dg_pitch = (target_gp - gimbal[0]) / SERVO_TAU
    dg_yaw = (target_gy - gimbal[1]) / SERVO_TAU
    
    # 2. Forces (Body Frame)
    # Body-Y is longitudinal.
    f_thrust_body = np.array([-thrust_mag * math.sin(gimbal[1]), 
                              thrust_mag * math.cos(gimbal[0]) * math.cos(gimbal[1]),
                              -thrust_mag * math.sin(gimbal[0])])
    
    # Wind and Relative Velocity
    wind_world = surface_wind + np.array([0, wind_shear_gradient * pos[1], 0])
    vel_rel_world = vel - wind_world
    vel_rel_mag = np.linalg.norm(vel_rel_world)
    
    # Parachute
    if vel[1] < 0 and t > burn_time:
        a_eff, cd_eff = parachute_area, parachute_cd
    else:
        a_eff, cd_eff = area_base, Cd_base
        
    f_drag_world = np.zeros(3)
    if vel_rel_mag > 1e-6:
        f_drag_mag = 0.5 * rho * cd_eff * a_eff * (vel_rel_mag**2)
        f_drag_world = - (vel_rel_world / vel_rel_mag) * f_drag_mag
        
    # Rotate forces to Body frame for torque or World frame for translation
    f_thrust_world = q_rotate_vec(quat, f_thrust_body)
    f_net_world = f_thrust_world + f_drag_world + (mass * gravity_vec)
    
    accel_world = f_net_world / mass
    
    # 3. Torques (Body Frame)
    # CG to base distance is cg
    r_tvc_body = np.array([0, -cg, 0])
    torque_tvc = np.cross(r_tvc_body, f_thrust_body)
    
    # Aerodynamic Restoration
    vel_rel_body = q_rotate_vec(q_conjugate(quat), vel_rel_world)
    torque_aero = np.zeros(3)
    if vel_rel_mag > 1e-6:
        dist_cg_cp = cp_from_base - cg
        # Radial restoration opposes lateral relative velocity components
        v_lateral = np.array([vel_rel_body[0], 0, vel_rel_body[2]])
        # restaura_force = 0.5 * rho * Cd * area * v_lateral * vel_rel_mag
        restoring_force_lateral = -0.5 * rho * Cd_base * area_base * v_lateral * vel_rel_mag
        torque_aero = np.cross(np.array([0, dist_cg_cp, 0]), restoring_force_lateral)

    torque_net_body = torque_tvc + torque_aero
    
    # 4. Rotational Dynamics (Euler Equation)
    Iw = I_tensor @ rates
    w_x_Iw = np.cross(rates, Iw)
    angular_accel = np.linalg.inv(I_tensor) @ (torque_net_body - w_x_Iw)
    
    # 5. Quaternion Kinematics
    # dq = 0.5 * q * [0, w]
    omega_quat = np.array([0, rates[0], rates[1], rates[2]])
    dq = 0.5 * q_mult(quat, omega_quat)
    
    return np.concatenate([vel, accel_world, dq, angular_accel, [dg_pitch, dg_yaw]])

def rk4_step(t, state, dt, err, rate, int_v):
    k1 = get_derivatives(t, state, err, rate, int_v)
    k2 = get_derivatives(t + dt/2, state + (dt/2) * k1, err, rate, int_v)
    k3 = get_derivatives(t + dt/2, state + (dt/2) * k2, err, rate, int_v)
    k4 = get_derivatives(t + dt, state + dt * k3, err, rate, int_v)
    return state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


# 4. Main Simulation Loop

# Initial State Vector (15 elements)
X = np.zeros(15)
X[6] = 1.0  # Identity Quaternion (Body-Y is World-Y)

dt = 0.01
time_elapsed = 0.0
est_error_vec = np.zeros(3)
est_rate_vec = np.zeros(3)
est_int_vec = np.zeros(3)

# Logging
log_t, log_pos, log_vel, log_quat, log_rates, log_gimbal = [], [], [], [], [], []
log_mass, log_thrust = [], []

print("Starting 6-DOF 3D Rocket Simulation (RK4 + Quaternions)...")

while time_elapsed == 0 or (X[1] >= 0 or time_elapsed < 0.5):
    # 1. IMU & State Estimation
    # Project World-UP [0, 1, 0] into Body frame
    quat_curr = q_normalize(X[6:10])
    v_up_body = q_rotate_vec(q_conjugate(quat_curr), np.array([0, 1, 0]))
    
    # Lateral components are error
    est_error_vec = np.array([v_up_body[0], 0.0, v_up_body[2]])
    
    # Noise on rates
    true_rates = X[10:13]
    est_rate_vec = true_rates + np.random.normal(0, 0.01, 3)
    
    # Integral update
    est_int_vec += est_error_vec * dt
    est_int_vec = np.clip(est_int_vec, -0.5, 0.5)

    # 2. Physics Advancement
    thrust_m, mass_m, _, _ = get_physics_properties(time_elapsed)
    
    log_t.append(time_elapsed)
    log_pos.append(X[0:3].copy())
    log_vel.append(X[3:6].copy())
    log_quat.append(X[6:10].copy())
    log_rates.append(X[10:13].copy())
    log_gimbal.append(X[13:15].copy())
    log_mass.append(mass_m)
    log_thrust.append(thrust_m)
    
    X = rk4_step(time_elapsed, X, dt, est_error_vec, est_rate_vec, est_int_vec)
    time_elapsed += dt
    
    if time_elapsed > 500: break

print("Simulation complete.")
log_pos = np.array(log_pos)
log_vel = np.array(log_vel)
log_gimbal = np.array(log_gimbal)
log_quat = np.array(log_quat)

apogee = np.max(log_pos[:,1])
print(f"Apogee: {apogee:.2f} m")
print(f"Total Time: {time_elapsed:.2f} s")


# 5. Visualization

fig = plt.figure(figsize=(15, 10))

# 3D Trajectory
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.plot(log_pos[:,0], log_pos[:,2], log_pos[:,1], 'b-', label='Trajectory')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Z (m)')
ax1.set_zlabel('Altitude (m)')
ax1.set_title('6-DOF 3D Flight Path')
ax1.grid(True)

# Altitude
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(log_t, log_pos[:,1], 'g-')
ax2.set_title('Altitude vs Time')
ax2.set_ylabel('m')
ax2.grid(True)

# Gimbals
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(log_t, np.degrees(log_gimbal[:,0]), 'r-', label='Pitch Gimbal')
ax3.plot(log_t, np.degrees(log_gimbal[:,1]), 'c-', label='Yaw Gimbal')
ax3.set_title('Dual-Axis TVC Angles')
ax3.set_ylabel('Degrees')
ax3.legend()
ax3.grid(True)

# Mass/Thrust
ax4 = fig.add_subplot(2, 2, 4)
ax4_t = ax4.twinx()
ax4.plot(log_t, log_mass, 'm-', label='Mass')
ax4_t.plot(log_t, log_thrust, 'orange', label='Thrust')
ax4.set_title('Dynamic Properties')
ax4.set_ylabel('Mass (kg)')
ax4_t.set_ylabel('Thrust (N)')
ax4.grid(True)

plt.tight_layout()
plt.savefig('~/pointy_rocket/simulation/simulation_results.png')
print("Static results saved to simulation_results.png")

# 3D Perspective Animation
fig_anim = plt.figure(figsize=(8, 8))
ax_anim = fig_anim.add_subplot(111, projection='3d')

def update_anim(frame):
    ax_anim.clear()
    idx = frame * 10 
    if idx >= len(log_pos): idx = len(log_pos)-1
    
    pos = log_pos[idx]
    q = q_normalize(log_quat[idx])
    
    # Body Axis Vector
    body_dir = q_rotate_vec(q, np.array([0, 1, 0])) # Body-Y is longitudinal
    tip = pos + body_dir * (height/2)
    base = pos - body_dir * (height/2)
    
    ax_anim.plot([base[0], tip[0]], [base[2], tip[2]], [base[1], tip[1]], 'blue', linewidth=5)
    
    # Ground
    lim = 50
    xx, zz = np.meshgrid(np.linspace(pos[0]-lim, pos[0]+lim, 5), np.linspace(pos[2]-lim, pos[2]+lim, 5))
    ax_anim.plot_surface(xx, zz, np.zeros_like(xx), alpha=0.1, color='green')
    
    ax_anim.set_title(f"T: {log_t[idx]:.2f}s | Alt: {pos[1]:.1f}m")
    ax_anim.set_xlim(pos[0]-20, pos[0]+20)
    ax_anim.set_ylim(pos[2]-20, pos[2]+20)
    ax_anim.set_zlim(pos[1]-5, pos[1]+40)

try:
    ani = FuncAnimation(fig_anim, update_anim, frames=len(log_t)//10, interval=50)
    # Save suppressed due to ffmpeg dependency
except:
    pass

plt.close('all')
