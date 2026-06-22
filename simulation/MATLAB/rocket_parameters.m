%% Rocket properties

mass = 0.891;        % kg

Iyy = 0.286;        % kg*m^2

CG = 0.518;         % m from nose

nozzle = 0.855;     % m from nose

back = 0.801;      % m

lever_arm = nozzle - CG;


%% TVC

max_gimbal = 8*pi/180;   % radians

servo_tau = 0.09;        % seconds


%% Simulation

dt = 0.001;

sim_time = 5;