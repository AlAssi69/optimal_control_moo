% MAIN_WORKFLOW - Optimization of Quarter-Car Suspension
clc; clear; close all;

% Add paths to libraries
addpath(genpath('../lib'));
addpath(genpath('../models'));
addpath(genpath('../')); % For config.m

%% 1. Setup Simulation Environment
disp('[1/6] Loading Configuration...');
conf = config();

disp('[2/6] Calculating System Matrices...');
[~, mats] = get_system_matrices(conf);

disp('[3/6] Generating Road Profile...');
[road_ts, zr, zr_dot, time_vec] = generate_road_profile(conf);

%% 2. Configure Optimization (MOPSO)
disp('[4/6] Configuring MOPSO Parameters...');

% A. Solver Parameters
params.Np = 50;          % Population Size (Higher = better search, slower)
params.Nr = 100;         % Repository/Archive Size
params.maxgen = 50;      % Generations (Increase to 50+ for real results)
params.W = 0.4;          % Inertia
params.C1 = 2.0;         % Cognitive
params.C2 = 2.0;         % Social
params.ngrid = 20;       % Grid divisions
params.maxvel = 5;       % Max velocity (%)
params.u_mut = 0.5;      % Mutation rate

% B. Problem Definition (Variables: Kp, Ki, Kd)
MultiObj.nVar = 3;
% Search Limits: [Kp, Ki, Kd]
%                 Kp    Ki   Kd
MultiObj.var_min = [1000, 0,   100];   % Lower Bounds
MultiObj.var_max = [50000, 500, 5000]; % Upper Bounds (Broad search space)

%% 3. Run Optimization
disp('------------------------------------------------');
disp('       STARTING MULTI-OBJECTIVE OPTIMIZATION    ');
disp('------------------------------------------------');
tic;

% Run the MOPSO function (Parallel version recommended: mopso_p)
try
    REP = mopso_p(conf, params, MultiObj, mats, road_ts);
catch
    warning('mopso_p not found, using standard mopso.');
    REP = mopso(conf, params, MultiObj, mats, road_ts);
end

elapsed_time = toc;
disp(['Optimization finished in ' num2str(elapsed_time/60) ' minutes.']);

%% 4. Select Key Solutions (Balanced, Comfort, Handling)
costs = REP.pos_fit;

% --- A. Balanced Solution (Closest to Utopia) ---
min_c = min(costs); max_c = max(costs);
norm_costs = (costs - min_c) ./ (max_c - min_c);

% B. Calculate distance to origin (0,0)
dist_to_zero = sqrt(sum(norm_costs.^2, 2));
[~, idx_bal] = min(dist_to_zero);

% --- B. Best Comfort (Min J1: ITAE Acceleration) ---
[~, idx_comf] = min(costs(:, 1));

% --- C. Best Handling (Min J2: ITAE Deflection) ---
[~, idx_hand] = min(costs(:, 2));

% --- Pack Data for Plotting ---
special_points.balanced.gains = REP.pos(idx_bal, :);
special_points.balanced.costs = REP.pos_fit(idx_bal, :);

special_points.comfort.gains = REP.pos(idx_comf, :);
special_points.comfort.costs = REP.pos_fit(idx_comf, :);

special_points.handling.gains = REP.pos(idx_hand, :);
special_points.handling.costs = REP.pos_fit(idx_hand, :);

disp('------------------------------------------------');
disp('             KEY SOLUTIONS SELECTED             ');
disp('------------------------------------------------');
disp('1. Balanced Solution:');
disp(['   Costs: ' num2str(special_points.balanced.costs)]);
disp('2. Best Comfort Solution (Min Accel):');
disp(['   Costs: ' num2str(special_points.comfort.costs)]);
disp('3. Best Handling Solution (Min Deflection):');
disp(['   Costs: ' num2str(special_points.handling.costs)]);

%% 5. Verify & Plot All Key Solutions
disp('[5/6] Validating and Plotting Key Solutions...');

% --- A. Balanced Solution ---
disp('   > Simulating Balanced Solution...');
gains = special_points.balanced.gains;
results_bal = run_simulation(conf, mats, road_ts, gains(1), gains(2), gains(3));

plot_simulation_results(results_bal, conf, zr, time_vec);
% Update Title to distinguish this plot
set(gcf, 'Name', 'Results: Balanced Solution');
sgtitle(['Balanced Solution (Kp=' num2str(gains(1)) ')'], 'Color', 'k');


% --- B. Best Comfort Solution ---
disp('   > Simulating Best Comfort Solution...');
gains = special_points.comfort.gains;
results_comf = run_simulation(conf, mats, road_ts, gains(1), gains(2), gains(3));

plot_simulation_results(results_comf, conf, zr, time_vec);
set(gcf, 'Name', 'Results: Best Comfort');
sgtitle(['Best Comfort Solution (Kp=' num2str(gains(1)) ')'], 'Color', 'k');


% --- C. Best Handling Solution ---
disp('   > Simulating Best Handling Solution...');
gains = special_points.handling.gains;
results_hand = run_simulation(conf, mats, road_ts, gains(1), gains(2), gains(3));

plot_simulation_results(results_hand, conf, zr, time_vec);
set(gcf, 'Name', 'Results: Best Handling');
sgtitle(['Best Handling Solution (Kp=' num2str(gains(1)) ')'], 'Color', 'k');

%% 6. Save Optimization Results
save_path = fullfile('../res', 'optimization_results.mat');

% Ensure directory exists
if ~exist('../res', 'dir')
    mkdir('../res');
end

% Save all key data: Repository, Points, Config, and the 3 distinct simulation results
save(save_path, 'REP', 'special_points', 'conf', ...
    'results_bal', 'results_comf', 'results_hand');

disp(['All results saved to: ' save_path]);

%% 7. Plot Pareto Front (Trade-off Curve)
% Pass the struct containing all three points
plot_pareto_front(REP, special_points);