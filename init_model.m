clc;
clear;
close all force;

%% SECTION 1: System Parameters
% These values approximate a standard passenger vehicle
ms = 250; % Sprung mass (kg)
mu = 45; % Unsprung mass (kg)
ks = 28000; % Suspension stiffness (N/m)
bs = 2500; % Suspension damping (N/s/m)
kt = 190000; % Tire stiffness (N/m)

%% SECTION 2: State-Space Matrices Generation
% Defining the matrix A (System Matrix)
% Rows correspond to the derivatives of the state vector x
A = [ 0, 1, 0, -1;
    -ks/ms, -bs/ms, 0, bs/ms;
    0, 0, 0, 1;
    ks/mu, bs/mu, -kt/mu, -bs/mu];

% Defining the matrix B (Input Matrix)
% Inputs: u1 = Road Velocity (z_r_dot), u2 = Control Force (F)
B = [ 0, 0;
    0, 1/ms;
    -1, 0;
    0, -1/mu];

% Defining the matrix C (Output Matrix)
% Let's observe: [Suspension Deflection; Sprung Accel; Body Position]
C = [ 1, 0, 0, 0; % Output 1: Suspension Deflection
    -ks/ms, -bs/ms, 0, bs/ms; % Output 2: Sprung Acceleration (Force/mass)
    0, 0, 1, 0]; % Output 3: Tire Deflection

% Defining the matrix D (Feedforward Matrix)
D = [ 0, 0;
    0, 1/ms;
    0, 0];

%% SECTION 3: Create System Object
% Create a state-space system for linear analysis if needed
sys_suspension = ss(A, B, C, D);
% Display confirmation
disp('Suspension Parameters and State-Space Matrices Loaded.');

%% SECTION 3.5: Controller Parameters (PID)
% Initial guess for the Active Suspension Controller
% We want to minimize Suspension Deflection (keep the car level)

Kp = 5000;   % Proportional Gain (Stiffness)
Ki = 100;    % Integral Gain (Eliminate steady-state drift)
Kd = 400;    % Derivative Gain (Damping - prevents overshoot)

disp('PID Controller Gains Loaded.');
%% SECTION 4: Generate Road Profile (Speed Bump)
% Simulation settings
T_end = 5; % Total simulation time (s)
dt = 0.001; % Time step (s)
time = 0:dt:T_end; % Time vector

% Bump Parameters
bump_height = 0.10; % 10 cm bump
bump_length = 1.0; % 1 meter long bump
car_speed_kmh = 30; % Car speed in km/h

% Convert speed to m/s
V = car_speed_kmh * (1000/3600);

% Calculate transit time over the bump
bump_duration = bump_length / V;
start_time = 0.5; % Hit the bump at 0.5 seconds
end_time = start_time + bump_duration;

% Initialize vectors
zr = zeros(size(time)); % Road Height (Meters) - For reference only
zr_dot = zeros(size(time)); % Road Velocity (m/s) - ACTUAL INPUT

% Create the Haversine Bump
% Formula: z = 0.5*H * (1 - cos(2*pi*x/L))
for i = 1:length(time)
    t = time(i);
    if t >= start_time && t <= end_time
        % Local time inside the bump
        t_local = t - start_time;
        
        % Omega for the sine wave
        omega = 2 * pi * V / bump_length;
        
        % Road Height (Position)
        zr(i) = 0.5 * bump_height * (1 - cos(omega * t_local));
        
        % Road Velocity (Derivative) -> Feeds into B Matrix!
        zr_dot(i) = 0.5 * bump_height * omega * sin(omega * t_local);
    end
end

% Create Timeseries Object for Simulink
% We bundle Time and Data together so Simulink can read it easily
road_input_data = timeseries(zr_dot', time);

disp(['Road Profile Generated: Speed Bump at ' num2str(car_speed_kmh) ' km/h']);