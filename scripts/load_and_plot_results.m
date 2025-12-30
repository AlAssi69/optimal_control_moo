% LOAD_AND_PLOT_RESULTS - Reloads saved optimization data and visualizes it
clc; clear; close all;

% Add paths to ensure plotting functions are available
addpath(genpath('../lib'));
addpath(genpath('../models'));

%% 1. Load Data
load_path = fullfile('../res', 'optimization_results.mat');

if ~exist(load_path, 'file')
    error('Results file not found: %s\nPlease run main_workflow.m first.', load_path);
end

disp(['[1/3] Loading results from: ' load_path]);
load(load_path, 'REP', 'special_points', 'conf', ...
    'results_bal', 'results_comf', 'results_hand');

%% 2. Regenerate Road Profile (Required for Plotting)
% We didn't save the road vector 'zr' to keep the file size small,
% so we recreate it quickly using the saved config.
disp('[2/3] Regenerating reference road profile...');
[~, zr, ~, time_vec] = generate_road_profile(conf);

%% 3. Generate Plots
disp('[3/3] Generating Figures...');

% --- A. Pareto Front ---
plot_pareto_front(REP, special_points);
set(gcf, 'Name', 'Pareto Front Optimization'); % Rename window

% --- B. Balanced Solution ---
plot_simulation_results(results_bal, conf, zr, time_vec);
set(gcf, 'Name', 'Results: Balanced Solution');
sgtitle(['Balanced Solution (Kp=' num2str(special_points.balanced.gains(1)) ')'], 'Color', 'k');

% --- C. Best Comfort Solution ---
plot_simulation_results(results_comf, conf, zr, time_vec);
set(gcf, 'Name', 'Results: Best Comfort');
sgtitle(['Best Comfort Solution (Kp=' num2str(special_points.comfort.gains(1)) ')'], 'Color', 'k');

% --- D. Best Handling Solution ---
plot_simulation_results(results_hand, conf, zr, time_vec);
set(gcf, 'Name', 'Results: Best Handling');
sgtitle(['Best Handling Solution (Kp=' num2str(special_points.handling.gains(1)) ')'], 'Color', 'k');

disp('Success: All figures have been recreated.');