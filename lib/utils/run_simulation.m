function sim_results = run_simulation(conf, mats, road_ts, Kp, Ki, Kd)
% RUN_SIMULATION - Configures and runs the Simulink model
% Automatically switches to a _2023a version if running MATLAB R2023a.

% --- 1. Version Check & Model Selection ---
% Get the current MATLAB release (e.g., '2023a', '2024b')
v = version('-release');

% Start with the default name from config
model_file = conf.model_name;

% If version is exactly 2023a, try to use the specific model
if strcmp(v, '2023a')
    % Construct the potential 2023a filename
    model_23a = [model_file '_2023a'];
    
    % Check if this file actually exists (Simulink models are type 4)
    if exist(model_23a, 'file') == 4
        model_file = model_23a;
        % disp(['Running R2023a specific model: ' model_file]);
    else
        warning('Matlab R2023a detected, but %s.slx not found. Using default.', model_23a);
    end
end

% --- 2. Create Simulation Input Object ---
% Now we create the object with the CORRECT model name
in = Simulink.SimulationInput(model_file);

% --- 3. Inject Variables (The Model Workspace) ---
in = in.setVariable('T_end', conf.T_end);

% Inject Matrices
in = in.setVariable('A', mats.A);
in = in.setVariable('B', mats.B);
in = in.setVariable('C', mats.C);
in = in.setVariable('D', mats.D);

% Inject Inputs & Controller
in = in.setVariable('road_input_data', road_ts);
in = in.setVariable('Kp', Kp);
in = in.setVariable('Ki', Ki);
in = in.setVariable('Kd', Kd);

% --- 4. Run Simulation ---
try
    out = sim(in);
catch ME
    error('Simulation Failed using model "%s": %s', model_file, ME.message);
end

% --- 5. Extract Data ---
try
    % Note: We use the signal names as defined in "Log Selected Signals"
    sim_results.time = out.logsout.get('Suspension Deflection').Values.Time;
    sim_results.sus_def = out.logsout.get('Suspension Deflection').Values.Data;
    sim_results.sprung_acc = out.logsout.get('Sprung Acceleration').Values.Data;
    sim_results.tire_def = out.logsout.get('Tire Deflection').Values.Data;
    
    try
        sim_results.force = out.logsout.get('Force').Values.Data;
    catch
        sim_results.force = zeros(size(sim_results.time));
    end
    
    % Store the input profile used for this specific run
    sim_results.road_input = road_ts.Data;
    
catch ME
    error('Error extracting signals: %s', ME.message);
end
end