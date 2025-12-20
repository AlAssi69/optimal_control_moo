function sim_results = run_simulation(conf, mats, road_ts, Kp, Ki, Kd)
    % RUN_SIMULATION - Configures and runs the Simulink model
    % Returns a standardized struct of results
    
    % 1. Create Simulation Input Object
    in = Simulink.SimulationInput(conf.model_name);
    
    % 2. Inject Variables (The Model Workspace)
    % FIX: We must define 'T_end' because the model settings use it!
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
    
    % OPTIONAL: If your Solver settings use 'dt' (Fixed-step size), inject it too:
    % in = in.setVariable('dt', conf.dt);

    % 3. Run Simulation
    out = sim(in);
    
    % 4. Extract Data (Standardize names)
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
        error('Error extracting signals: %s\nMake sure "Suspension Deflection" and others are logged in the model.', ME.message);
    end
end