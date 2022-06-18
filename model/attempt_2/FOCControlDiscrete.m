% - Allow the motor analog section to run at whatever sampling rate
% - Run the current controller loop at 1 sample rate (HW)
% - Run the speed controller at 5x the rate of the current
% - Minimize the number of PID/IIR blocks to simplify the code

% Set constants for the FOC simulation
Vdc = 12;           % DC Link voltage
Ls = 2e-04;        % Stator inductance
Rs = 0.36;          % Stator Resistance
Psi = 0.006395415186893;         % Permanent magnet flux
P = 6;              % Number of poles
W_max_rpm = 30000; 

% PID Speed Controller
Kp_speed = 0.459;
Ki_speed = 6.0;

% PID Current Controller
Kp_current = 12.0;
Ki_current = 6.0;

% Sampling Domains
Ts_current_control_loop = 30e-6;
Ts_observer = 2 * Ts_current_control_loop;
Ts_speed_control_loop = 2 * Ts_observer;