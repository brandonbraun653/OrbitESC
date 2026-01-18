idleRotationRate = convangvel(983, 'rpm', 'rad/s');

% Si Unit Base Values for PU conversions
baseValue.adc_current_max = 16.0;    % Amps
baseValue.motor_omega     = 523.6;   % Rad/s (5000 rpm)
baseValue.supply_voltage  = 12.0;    % Volts

adcParams.iSenseLimit = 16.0;
adcParams.num_bits = 12;

% Motor parameters structure
motorParams.ts = 5e-6;        % Sample time (seconds) - 10kHz control loop
motorParams.p  = 8;            % Number of pole pairs
motorParams.rs = 10e-3;       % Stator resistance (Ohms)
motorParams.kv = 1000;         % Motor RPM/V rating
motorParams.ke = (60*sqrt(2))/motorParams.kv;       % Back EMF constant (Vpk_LL/krpm)
motorParams.inertia = 2.5e-6;  % Rotor inertia (kg·m²)
motorParams.damping = 4e-6;
motorParams.staticFriction = 1e-6;

motorParams.ldq = 7.32e-6;     % d-axis inductance (Henrys)
motorParams.ldq_pu = motorParams.ldq / (baseValue.supply_voltage / (baseValue.motor_omega * baseValue.adc_current_max));

motorParams.flux_linkage = 60 / (motorParams.kv * motorParams.p * pi * sqrt(2));
motorParams.flux_linkage_pu = motorParams.flux_linkage / (baseValue.supply_voltage / baseValue.motor_omega);

% Inner loop current control settings:
% When setting PID bandwidth, a general rule of thumb is sample
% freq is 10x-20x larger than control bandwidth.
ccFreqHz = 20e3;
ccTs = 1/ccFreqHz;  % Control cycle sample time (seconds)
omega_bw_hz = ccFreqHz / 10;
omega_bw = 2*pi*omega_bw_hz; % Bandwidth in rad/s

dq_pid.satUpper = 1.0;  % Upper saturation limit
dq_pid.satLower = -1.0; % Lower saturation limit

dq_pid.Kp = omega_bw*motorParams.ldq;
dq_pid.Kp_pu = dq_pid.Kp * (baseValue.adc_current_max / baseValue.supply_voltage);

dq_pid.Ki = omega_bw*motorParams.rs;
dq_pid.Ki_pu = dq_pid.Ki * (baseValue.adc_current_max / baseValue.supply_voltage);

% Ramp parameters
rampParams.rate = 500;   % Ramp rate, per-unit
rampParams.id = 0.0;
rampParams.iq = 0.95;

% TCP/IP communication setting
useTCP = false;