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
params.Np = 5;          % Population Size (Higher = better search, slower)
params.Nr = 100;         % Repository/Archive Size
params.maxgen = 5;      % Generations (Increase to 50+ for real results)
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
tic; % Start timer

% Run the MOPSO function
% Note: This implicitly calls run_simulation() and calculate_objectives()
REP = mopso_p(conf, params, MultiObj, mats, road_ts);

elapsed_time = toc;
disp(['Optimization finished in ' num2str(elapsed_time/60) ' minutes.']);

%% 4. Select a "Balanced" Solution
% The Repository (REP) contains many solutions. We need to pick one to visualize.
% Strategy: Find the solution closest to the "Utopia Point" (0,0) (Normalized)

% A. Normalize the objectives to 0-1 range to compare fairly
costs = REP.pos_fit;
min_c = min(costs); max_c = max(costs);
norm_costs = (costs - min_c) ./ (max_c - min_c);

% B. Calculate distance to origin (0,0)
dist_to_zero = sqrt(sum(norm_costs.^2, 2));

% C. Find index of minimum distance
[~, idx_best] = min(dist_to_zero);
best_gains = REP.pos(idx_best, :);
best_costs = REP.pos_fit(idx_best, :);

disp('------------------------------------------------');
disp('             OPTIMAL SOLUTION SELECTED          ');
disp('------------------------------------------------');
disp(['Kp: ' num2str(best_gains(1))]);
disp(['Ki: ' num2str(best_gains(2))]);
disp(['Kd: ' num2str(best_gains(3))]);
disp(['Comfort Cost (ITAE Acc): ' num2str(best_costs(1))]);
disp(['Handling Cost (ITAE Def): ' num2str(best_costs(2))]);

%% 5. Verify & Plot the Best Solution
disp('[5/6] Running Validation Simulation...');

% Run simulation with the Optimized Gains
results = run_simulation(conf, mats, road_ts, best_gains(1), best_gains(2), best_gains(3));

% Plot using your reusable function
disp('[6/6] Plotting Results...');
plot_simulation_results(results, conf, zr, time_vec);

%% 6. Save Optimization Results
save_path = fullfile('../res', 'optimization_results.mat');
if ~exist('../res', 'dir'), mkdir('../res'); end
save(save_path, 'REP', 'best_gains', 'results', 'conf');
disp(['All results saved to: ' save_path]);

%% 7. Plot Pareto Front (Trade-off Curve)
plot_pareto_front(REP, best_costs);