function conf = config()
    % CONFIG - Returns a struct containing all system parameters
    
    % --- 1. Vehicle Parameters ---
    conf.ms = 250;      % Sprung mass (kg)
    conf.mu = 45;       % Unsprung mass (kg)
    conf.ks = 28000;    % Suspension stiffness (N/m)
    conf.bs = 2500;     % Suspension damping (N/s/m)
    conf.kt = 190000;   % Tire stiffness (N/m)
    
    % --- 2. Simulation Settings ---
    conf.model_name = 'pid_tuning_moo'; % Name of the slx file
    conf.T_end = 5;         % Total simulation time (s)
    conf.dt = 0.001;        % Time step (s)
    
    % --- 3. Road Profile Settings ---
    conf.road_type = 'parabolic'; % 'parabolic' or 'haversine'
    conf.bump_height = 0.15;      % Meters
    conf.bump_length = 5.0;       % Meters
    conf.car_speed_kmh = 30;      % km/h
    conf.start_time = 1.0;        % Impact time (s)
    
    % --- 4. Default Controller Gains (Baseline) ---
    conf.Kp = 5000;
    conf.Ki = 100;
    conf.Kd = 400;
end