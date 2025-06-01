clear 
clc
close

%% Define constants

x0 = [85;    % Initial forward velocity (m/s)
    0;      % Initial lateral velocity (m/s)
    0;      % Initial vertical velocity (m/s)
    0;      % Initial roll rate (rad/s)
    0;      % Initial pitch rate (rad/s)
    0;      % Initial yaw rate (rad/s)
    0;      % Initial roll angle (rad)
    0.01;    % Initial pitch angle (rad) (5.73 degrees)
    0];     % Initial yaw angle (rad)

u = [0.0;    % Aileron deflection (rad)
    0.0;    % Elevator deflection (rad)
    0.0;     % Rudder deflection (rad)
    0.08];   % Throttle setting (normalized)

% TF: Simulation time (seconds)
TF = 60*3; % 3 minutes

% aileron command
u1min = -25 * pi/180;
u1max = 25 * pi/180;

% tail command
u2min = -25 * pi/180;
u2max = 10 * pi/180;

% rudder command
u3min = -30 * pi/180;
u3max = 30 * pi/180;

% throttle 1 command
u4min = 0.5 * pi/180;
u4max = 10 * pi/180;

% throttle 2 command
u5min = 0.5 * pi/180;
u5max = 10 * pi/180;
%% compute trim 
params.u1min = u1min; % Aileron min deflection
params.u1max = u1max; % Aileron max deflection
params.u2min = u2min; % Stabilator min deflection
params.u2max = u2max; % Stabilator max deflection
params.u3min = u3min; % Rudder min deflection
params.u3max = u3max; % Rudder max deflection
params.u4min = u4min; % Throttle 1 min setting
params.u4max = u4max; % Throttle 1 max setting
params.u5min = u5min; % Throttle 2 min setting
params.u5max = u5max; % Throttle 2 max setting
u_trim = computeTrim(x0, params);
disp('Trimmed Control Inputs:')
disp(u_trim);
u = u_trim(1:end-1);

%% linearized model 
[A, B, C, D] = linmod('gncTaira307', x0, u);
sys = ss(A, B, C, D);
s
%% Run the model

sim('gncTaira307.slx')

%% vis
% Extract time, states, and inputs
out = ans;
t = out.tout;
X = out.simX.Data;
U = out.simU.Data;

% Extract Euler angles from state vector
phi   = X(:,7); % roll
theta = X(:,8); % pitch
psi   = X(:,9); % yaw

% Extract control inputs
aileron    = U(:,1);
stabilator = U(:,2);
rudder     = U(:,3);
throttle1  = U(:,4);
throttle2  = U(:,5);

% Plot control inputs
figure;
subplot(3,1,1);
plot(t, aileron, 'r', t, stabilator, 'b', t, rudder, 'g');
xlabel('Time (s)');
ylabel('Control Surface Deflection (rad)');
legend('Aileron', 'Stabilator', 'Rudder');
title('Control Surface Commands');

subplot(3,1,2);
plot(t, throttle1, 'm', t, throttle2, 'c');
xlabel('Time (s)');
ylabel('Throttle Command');
legend('Throttle 1', 'Throttle 2');
title('Engine Commands');

% Plot aircraft orientation (Euler angles)
subplot(3,1,3);
plot(t, rad2deg(phi), 'r', t, rad2deg(theta), 'g', t, rad2deg(psi), 'b');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Roll \phi', 'Pitch \theta', 'Yaw \psi');
title('Aircraft Orientation (Euler Angles)');
%% vis 2

% Extract time, control inputs
t = out.simU.Time;
U = out.simU.Data;

aileron    = U(:,1); % d_A
stabilator = U(:,2); % d_T
rudder     = U(:,3); % d_R
throttle1  = U(:,4); % d_th1
throttle2  = U(:,5); % d_th2

% Plot control inputs
figure;
subplot(5,1,1);
plot(t, aileron, 'r');
ylabel('Aileron (rad)');
title('Control Inputs');

subplot(5,1,2);
plot(t, stabilator, 'b');
ylabel('Stabilator (rad)');

subplot(5,1,3);
plot(t, rudder, 'g');
ylabel('Rudder (rad)');

subplot(5,1,4);
plot(t, throttle1, 'm');
ylabel('Throttle 1');

subplot(5,1,5);
plot(t, throttle2, 'c');
ylabel('Throttle 2');
xlabel('Time (s)');

% Extract time and state data
t = out.simX.Time;
X = out.simX.Data;

u     = X(:,1);
v     = X(:,2);
w     = X(:,3);
p     = X(:,4);
q     = X(:,5);
r     = X(:,6);
phi   = X(:,7);  % roll
theta = X(:,8);  % pitch
psi   = X(:,9);  % yaw

% Plot states
figure;
subplot(3,3,1);
plot(t, u); ylabel('u (m/s)'); title('Forward Velocity');

subplot(3,3,2);
plot(t, v); ylabel('v (m/s)'); title('Lateral Velocity');

subplot(3,3,3);
plot(t, w); ylabel('w (m/s)'); title('Vertical Velocity');

subplot(3,3,4);
plot(t, p); ylabel('p (rad/s)'); title('Roll Rate');

subplot(3,3,5);
plot(t, q); ylabel('q (rad/s)'); title('Pitch Rate');

subplot(3,3,6);
plot(t, r); ylabel('r (rad/s)'); title('Yaw Rate');

subplot(3,3,7);
plot(t, rad2deg(phi)); ylabel('\phi (deg)'); title('Roll Angle');

subplot(3,3,8);
plot(t, rad2deg(theta)); ylabel('\theta (deg)'); title('Pitch Angle');

subplot(3,3,9);
plot(t, rad2deg(psi)); ylabel('\psi (deg)'); title('Yaw Angle');

xlabel('Time (s)');

% Function to compute trim values for a given state
function [u_trim] = computeTrim(x_trim, params)
    % Inputs:
    % x_trim: Desired state vector [u, v, w, p, q, r, phi, theta, psi]
    % params: Aircraft parameters (e.g., aerodynamic coefficients, mass, inertia)

    % Outputs:
    % u_trim: Control input vector [aileron, stabilator, rudder, throttle1, throttle2]

    % Define optimization problem
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

    % Initial guess for control inputs
    u0 = [0; 0; 0; 0.08; 0.08]; % Neutral control inputs

    % Define bounds for control inputs
    u_min = [params.u1min; params.u2min; params.u3min; params.u4min; params.u5min];
    u_max = [params.u1max; params.u2max; params.u3max; params.u4max; params.u5max];

    % Objective function: Minimize deviation from steady-state forces and moments
    objective = @(u) trimObjective(u, x_trim, params);

    % Solve optimization problem
    u_trim = fmincon(objective, u0, [], [], [], [], u_min, u_max, [], options);
end

% Objective function for trim analysis
function cost = trimObjective(u, x_trim, params)
    % Simulate aircraft dynamics with given state and control inputs
    % Compute forces and moments using FixedWingPlaneModel
    XDOT = FixedWingPlaneModel(x_trim, u);

    % Extract forces and moments from state derivatives (XDOT)
    forces = XDOT(1:3); % [udot, vdot, wdot]
    moments = XDOT(4:6); % [pdot, qdot, rdot]

    % Compute cost as the sum of squared deviations from zero forces and moments
    cost = sum(forces.^2) + sum(moments.^2);
end

% Placeholder for aircraft dynamics function
function [forces, moments] = aircraftDynamics(x, u, params)
    % Compute aerodynamic, thrust, and gravitational forces and moments
    % This function should be implemented based on the aircraft model

    forces = [0; 0; 0]; % Placeholder
    moments = [0; 0; 0]; % Placeholder
end
