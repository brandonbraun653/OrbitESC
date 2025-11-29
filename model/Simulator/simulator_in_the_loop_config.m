idleRotationRate = convangvel(1000, 'rpm', 'rad/s');

adcParams.iSenseLimit = 16.0;

% Motor parameters structure
motorParams.ts = 5e-6;        % Sample time (seconds) - 10kHz control loop
motorParams.p = 8;            % Number of pole pairs
motorParams.rs = 10e-3;       % Stator resistance (Ohms)
motorParams.ldq = 7.32e-6;     % d-axis inductance (Henrys)
motorParams.kv = 1000;         % Motor RPM/V rating
motorParams.ke = (60*sqrt(2))/motorParams.kv;       % Back EMF constant (Vpk_LL/krpm)
motorParams.inertia = 2.5e-6;  % Rotor inertia (kg·m²)
motorParams.damping = 4e-6;
motorParams.staticFriction = 1e-6;

% Control cycle sample time
ccTs = 30e-6;  % Control cycle sample time (seconds)

% DQ-axis PID saturation limits
dq_pid.satUpper = 1.0;  % Upper saturation limit
dq_pid.satLower = -1.0; % Lower saturation limit
dq_pid.Kp = 0.001;
dq_pid.Ki = 120;
dq_pid.Kd = 0.0;

% Ramp parameters
rampParams.rate = 1000;   % Ramp rate/slope
rampParams.id = 0.0;
rampParams.iq = 0.8;

% TCP/IP communication setting
useTCP = false;