clc;
clear;
close all force;
bdclose('all');

init_model;

%% SECTION 5: Run Simulation & Extract Data
% Define the model name (Change this to match your actual Simulink filename)
model_name = 'pid_tuning_moo';

% Run the simulation from the script
disp('Running Simulation...');
out = sim(model_name);

disp('Simulation Complete. Plotting Results...');

% Extract Data (Assumes you used "Log Selected Signals")
% Modern Simulink stores logs in 'out.logsout'.
% We access them by the name of the signal or the block it comes from.
% NOTE: If these names don't match, check your signal names in Simulink!

try
    % Time vector
    t_sim = out.logsout.get('Suspension Deflection').Values.Time;
    
    % Signals
    sus_def = out.logsout.get('Suspension Deflection').Values.Data;
    sprung_acc = out.logsout.get('Sprung Acceleration').Values.Data;
    tire_def = out.logsout.get('Tire Deflection').Values.Data;
    
    % Try to get Force if logged, otherwise ignore
    try
        ctrl_force = out.logsout.get('Force').Values.Data;
    catch
        ctrl_force = zeros(size(t_sim)); % Placeholder if not logged
    end
    
    % Input (Road Profile) - We can just use the workspace variable
    % Resample 'zr_dot' to match simulation time if needed, or just plot the input array
    
catch
    error('Could not find signals! Make sure you right-clicked signal lines in Simulink and selected "Log Selected Signals".');
end

%% SECTION 6: Generate Subplot Figure
figure('Name', 'Quarter-Car Active Suspension Results', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% --- Subplot 1: Road Input ---
subplot(2, 2, 1);
plot(time, zr, 'r--', 'LineWidth', 1.5); hold on;
% Scale velocity to be visible or plot on right axis, but let's stick to Height
ylabel('Road Height (m)');
title('Road Profile Input');
grid on;
legend('Road Height (z_r)');
xlim([0, T_end]);

% --- Subplot 2: Suspension Deflection ---
subplot(2, 2, 2);
plot(t_sim, sus_def, 'k', 'LineWidth', 1.5);
ylabel('Deflection (m)');
title('Suspension Deflection (z_s - z_u)');
grid on;
yline(0, 'Color', [0.5 0.5 0.5], 'LineStyle', '--'); % Zero reference
xlim([0, T_end]);

% --- Subplot 3: Sprung Mass Acceleration (Comfort) ---
subplot(2, 2, 3);
plot(t_sim, sprung_acc, 'b', 'LineWidth', 1.5);
ylabel('Acceleration (m/s^2)');
title('Sprung Mass Acceleration (Comfort)');
grid on;
xlim([0, T_end]);

% --- Subplot 4: Control Force ---
subplot(2, 2, 4);
plot(t_sim, ctrl_force, 'g', 'LineWidth', 1.5);
ylabel('Force (N)');
title('Active Control Force');
grid on;
xlim([0, T_end]);

% Add a main title to the whole figure
sgtitle(['Active Suspension Response at ' num2str(car_speed_kmh) ' km/h']);