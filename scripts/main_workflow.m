% MAIN_WORKFLOW - Orchestrates the Quarter-Car Simulation
clc; clear; close all;

% Add paths
addpath(genpath('../lib'));
addpath(genpath('../models'));
addpath(genpath('../')); % For config.m

%% 1. Load Configuration & Prepare System
disp('[1/5] Loading Configuration...');
conf = config();

disp('[2/5] Calculating System Matrices...');
[~, mats] = get_system_matrices(conf);

disp('[3/5] Generating Road Profile...');
[road_ts, zr, zr_dot, time_vec] = generate_road_profile(conf);

%% 2. Run Simulation (Baseline PID)
disp(['[4/5] Running Simulation with Baseline PID: ' ...
      'Kp=' num2str(conf.Kp) ', Ki=' num2str(conf.Ki) ', Kd=' num2str(conf.Kd)]);

% Run the wrapper function
results = run_simulation(conf, mats, road_ts, conf.Kp, conf.Ki, conf.Kd);

% Calculate Objectives
[J_comfort, J_handling] = calculate_objectives(results);
disp(['      > Comfort Cost (RMS Accel): ' num2str(J_comfort)]);
disp(['      > Handling Cost (RMS Def):  ' num2str(J_handling)]);

%% 3. Save Results
save_filename = 'baseline_results.mat';
target_folder = '../res'; % Relative path to the res folder

% Combine them to get the full path
save_path = fullfile(target_folder, save_filename);

% Check if the directory exists; if not, create it
if ~exist(target_folder, 'dir')
    mkdir(target_folder);
    disp(['Directory not found. Created new folder: ' target_folder]);
end

% Save the data
save(save_path, 'results', 'conf', 'zr', 'zr_dot', 'time_vec');
disp(['[5/5] Results saved to: ' save_path]);

%% 4. Plot Results
% Use the reusable function from lib/utils
plot_simulation_results(results, conf, zr, time_vec);