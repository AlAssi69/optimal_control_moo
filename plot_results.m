clc;
clear;
close all force;
bdclose('all');

% Run the initialization if needed
try
    init_model;
catch
    warning('init_model script not found. Using existing workspace variables.');
end

%% SECTION 5: Run Simulation & Extract Data
model_name = 'pid_tuning_moo';

disp('Running Simulation...');
out = sim(model_name);
disp('Simulation Complete. Plotting Results...');

try
    % --- 1. Extract Time ---
    % Get simulation time vector from logs
    t_sim = out.logsout.get('Suspension Deflection').Values.Time;
    
    % --- 2. Extract Logged Signals (Wi-Fi Icons) ---
    sus_def    = out.logsout.get('Suspension Deflection').Values.Data;
    sprung_acc = out.logsout.get('Sprung Acceleration').Values.Data;
    tire_def   = out.logsout.get('Tire Deflection').Values.Data;
    
    % Force (Output of Saturation Block)
    try
        ctrl_force = out.logsout.get('Force').Values.Data;
    catch
        ctrl_force = zeros(size(t_sim));
        warning('Force signal not found in logs.');
    end
    
    % Road Velocity (Input Block)
    try
        road_vel_log = out.logsout.get('road_input_data').Values.Data;
    catch
        % Fallback to workspace variable if log is missing
        road_vel_log = zr_dot;
    end
    
catch
    error('Could not find signals! Check signal names in Simulink and ensure "Log Selected Signals" is checked.');
end

%% SECTION 6: Generate Subplot Figure (6 Plots)
figure('Name', 'Quarter-Car Active Suspension Results', 'Color', 'w', 'Position', [100, 50, 1200, 900]);

% --- Plot 1: Road Height (Visual Reference) ---
subplot(3, 2, 1);
plot(time, zr, 'r--', 'LineWidth', 1.5);
ylabel('Height (m)');
title('1. Road Profile Height (z_r)');
grid on;
xlim([0, T_end]);

% --- Plot 2: Road Velocity (Actual System Input) ---
subplot(3, 2, 2);
% SMART PLOT: Check if data length matches simulation time or workspace time
if length(road_vel_log) == length(t_sim)
    plot(t_sim, road_vel_log, 'r', 'LineWidth', 1.5);
else
    % Fallback: If lengths differ, it must be the workspace variable
    plot(time, road_vel_log, 'r', 'LineWidth', 1.5);
end
ylabel('Velocity (m/s)');
title('2. Road Velocity Input (z_r dot)');
grid on;
xlim([0, T_end]);

% --- Plot 3: Suspension Deflection ---
subplot(3, 2, 3);
plot(t_sim, sus_def, 'k', 'LineWidth', 1.5);
ylabel('Deflection (m)');
title('3. Suspension Deflection (z_s - z_u)');
grid on;
yline(0, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
xlim([0, T_end]);

% --- Plot 4: Tire Deflection ---
subplot(3, 2, 4);
plot(t_sim, tire_def, 'm', 'LineWidth', 1.5);
ylabel('Deflection (m)');
title('4. Tire Deflection (z_u - z_r)');
grid on;
xlim([0, T_end]);

% --- Plot 5: Sprung Mass Acceleration (Comfort) ---
subplot(3, 2, 5);
plot(t_sim, sprung_acc, 'b', 'LineWidth', 1.5);
ylabel('Acceleration (m/s^2)');
title('5. Sprung Mass Acceleration (Comfort)');
grid on;
xlim([0, T_end]);

% --- Plot 6: Control Force ---
subplot(3, 2, 6);
plot(t_sim, ctrl_force, 'g', 'LineWidth', 1.5);
ylabel('Force (N)');
title('6. Active Control Force');
grid on;
xlim([0, T_end]);

% Add Main Title
sgtitle(['Active Suspension Response at ' num2str(car_speed_kmh) ' km/h']);