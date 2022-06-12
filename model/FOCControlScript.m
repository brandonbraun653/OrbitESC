% Set constants for the FOC simulation
Ls = 0.00022;   % Stator inductance
Rs = 0.013;     % Stator Resistance
Psi = 0.03;     % Permanent magnet flux
gamma = 50000;  % Observer gain
P = 6;          % Number of poles
Vdc = 12;       % DC Link voltage

% PID Speed Controller
Kp_speed = 0.459;
Ki_speed = 6.0;
Kd_speed = 0.0;

% Sample rates 
Ts = 50e-6;
Ts_motor = 25e-6;
Ts_inverter = 25e-6;
Ts_speed = 1e-3;

