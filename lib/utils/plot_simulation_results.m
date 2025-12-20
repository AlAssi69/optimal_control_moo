function plot_simulation_results(results, conf, zr, time_vec)
    % PLOT_SIMULATION_RESULTS - Visualizes Quarter-Car simulation data
    %
    % Usage:
    %   plot_simulation_results() 
    %       -> Automatically loads '../res/baseline_results.mat'
    %   plot_simulation_results(results, conf, zr, time_vec)
    %       -> Uses the data passed as arguments
    
    % --- 1. Handle Input Arguments ---
    if nargin == 0
        % If no inputs provided, try loading the default saved file
        default_file = '../res/baseline_results.mat';
        if exist(default_file, 'file')
            disp(['No inputs provided. Loading default: ' default_file]);
            loaded_data = load(default_file);
            
            % Unpack variables from the file
            results = loaded_data.results;
            conf = loaded_data.conf;
            zr = loaded_data.zr;
            time_vec = loaded_data.time_vec;
        else
            error('No input arguments provided and default file (../res/baseline_results.mat) not found.');
        end
    elseif nargin < 4
        error('Insufficient arguments. Usage: plot_simulation_results(results, conf, zr, time_vec)');
    end

    % --- 2. Generate Figures ---
    figure('Name', 'Quarter-Car Results', 'Color', 'w', 'Position', [100, 50, 1200, 900]);

    % Plot 1: Road Height (Visual Reference)
    subplot(3, 2, 1);
    plot(time_vec, zr, 'r--', 'LineWidth', 1.5);
    title('1. Road Profile Height'); 
    ylabel('Height (m)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Plot 2: Road Velocity (Actual Input)
    subplot(3, 2, 2);
    if length(results.road_input) == length(results.time)
        % Case A: Data came from Simulink logs (Variable step)
        plot(results.time, results.road_input, 'r', 'LineWidth', 1.5);
    elseif length(results.road_input) == length(time_vec)
        % Case B: Data came from Workspace (Fixed step) - MOST LIKELY
        plot(time_vec, results.road_input, 'r', 'LineWidth', 1.5);
    else
        % Fallback: Force interpolation if lengths match neither
        warning('Vector mismatch in Plot 2. Interpolating for display.');
        interp_road = linspace(0, conf.T_end, length(results.road_input));
        plot(interp_road, results.road_input, 'r', 'LineWidth', 1.5);
    end
    title('2. Road Velocity Input'); 
    ylabel('Velocity (m/s)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Plot 3: Suspension Deflection
    subplot(3, 2, 3);
    plot(results.time, results.sus_def, 'k', 'LineWidth', 1.5);
    title('3. Suspension Deflection'); 
    ylabel('Deflection (m)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Plot 4: Tire Deflection
    subplot(3, 2, 4);
    plot(results.time, results.tire_def, 'm', 'LineWidth', 1.5);
    title('4. Tire Deflection'); 
    ylabel('Deflection (m)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Plot 5: Sprung Acceleration (Comfort)
    subplot(3, 2, 5);
    acc_rms = rms(results.sprung_acc);
    plot(results.time, results.sprung_acc, 'b', 'LineWidth', 1.5);
    title(['5. Acceleration (RMS: ' num2str(acc_rms, '%.3f') ')']); 
    ylabel('Acceleration (m/s^2)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Plot 6: Control Force
    subplot(3, 2, 6);
    plot(results.time, results.force, 'g', 'LineWidth', 1.5);
    title('6. Control Force'); 
    ylabel('Force (N)'); 
    grid on; 
    xlim([0, conf.T_end]);

    % Main Title
    sgtitle(['Simulation Results: ' conf.road_type ' bump at ' num2str(conf.car_speed_kmh) ' km/h']);
end