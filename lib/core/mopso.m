% mopso performs Multi-Objective Particle Swarm Optimization over continuous functions.
%
% Input parameters:
%   params: Struct with customized parameters:
%       - Np:        Number of particles.
%       - Nr:        Repository size (archive size).
%       - maxgen:    Maximum number of generations.
%       - W:         Inertia coefficient.
%       - C1:        Personal (cognitive) confidence factor.
%       - C2:        Social (swarm) confidence factor.
%       - ngrid:     Number of hypercubes (grid divisions) in each dimension.
%       - maxvel:    Maximum velocity (as percentage of search space range).
%       - u_mut:     Uniform mutation percentage.
%
%   MultiObj: Struct with problem-specific parameters:
%       - nVar:      Number of decision variables.
%       - var_min:   Lower bounds of the search space.
%       - var_max:   Upper bounds of the search space.
%
% Output:
%   REP: Repository containing the non-dominated (Pareto-optimal) solutions.

% rng('default');

plot_figure = false;

%% Set MOPSO parameters
Np = particles;                 % Population size
Nr = 100;                       % Repository size (archive for non-dominated solutions)
maxgen = Max_iteration;         % Maximum number of generations
W = 0.4;                        % Inertia weight for velocity update
C1 = 2;                         % Cognitive (personal) confidence factor
C2 = 2;                         % Social (swarm) confidence factor
ngrid = 20;                     % Number of grids (hypercubes) in each objective dimension
maxvel = 5;                     % Maximum velocity (percentage of the search space)
u_mut = 0.5;                    % Uniform mutation percentage

nVar    = dim;
var_min = reshape(lb, 1, []);   % Lower bounds for kp, ki, kd
var_max = reshape(ub, 1, []);   % Upper bounds for kp, ki, kd

%% Initialization of Particles
% Randomly initialize particle positions within the search space
POS = repmat((var_max - var_min), Np, 1) .* rand(Np, nVar) + repmat(var_min, Np, 1);
VEL = zeros(Np, nVar);       % Initialize velocities to zero

% Preallocate POS_fit with shape Np x nObj (each particle's objective values)
nObj = 2;
POS_fit = zeros(Np, nObj);

% Loop through each particle and evaluate the objective function
for i = 1:size(POS, 1)
    vals = num2cell(POS(i, :));   % Convert vector to cell array
    
    [kp, ki, lam, kpi, kii, lami] = deal(vals{:});  % Unpack
    
    % Assign to base workspace for Simulink
    assignin('base', 'kp', kp);
    assignin('base', 'ki', ki);
    assignin('base', 'lam', lam);
    assignin('base', 'kpi', kpi);
    assignin('base', 'kii', kii);
    assignin('base', 'lami', lami);
    
    gen = 0;
    
    OptimizationModel;
    
    % objVal = [IATE_transient, IATE_steady];  % Expected to return a 2x1 vector
    objVal = [rmse_per_time, settling_time];
    % objVal = [overshoot, rmse_per_time];
    % objVal = [overshoot, settling_time];
    
    % Store the output as a row in POS_fit (transpose to a row vector)
    POS_fit(i, :) = objVal(:)';
end

% Check if each particle has a fitness value
if size(POS, 1) ~= size(POS_fit, 1)
    warning(['The objective function is not returning a value for each particle.' ...
        ' Please check its implementation.']);
end

% Set initial personal best positions and fitness
PBEST     = POS;
PBEST_fit = POS_fit;

% Initialize repository (archive) with non-dominated solutions
DOMINATED = checkDomination(POS_fit);
REP.pos      = POS(~DOMINATED, :);
REP.pos_fit  = POS_fit(~DOMINATED, :);
REP = updateGrid(REP, ngrid);

% Adjust maximum velocity based on search space range
maxvel = (var_max - var_min) .* maxvel ./ 100;
gen = 1;

%% Plotting Initialization (if 2 or 3 objectives)
if plot_figure == true
    if size(POS_fit, 2) == 2
        h_fig = figure(2);
        h_par = plot(POS_fit(:,1), POS_fit(:,2), 'or'); hold on;
        h_rep = plot(REP.pos_fit(:,1), REP.pos_fit(:,2), 'ok'); hold on;
        try %#ok<*TRYNC>
            set(gca, 'xtick', REP.hypercube_limits(:,1)', 'ytick', REP.hypercube_limits(:,2)');
            axis([min(REP.hypercube_limits(:,1)) max(REP.hypercube_limits(:,1)) ...
                min(REP.hypercube_limits(:,2)) max(REP.hypercube_limits(:,2))]);
            grid on; xlabel('f1'); ylabel('f2');
        end
        drawnow;
    end
    if size(POS_fit, 2) == 3
        h_fig = figure(2);
        h_par = plot3(POS_fit(:,1), POS_fit(:,2), POS_fit(:,3), 'or'); hold on;
        h_rep = plot3(REP.pos_fit(:,1), REP.pos_fit(:,2), REP.pos_fit(:,3), 'ok'); hold on;
        try
            set(gca, 'xtick', REP.hypercube_limits(:,1)', 'ytick', REP.hypercube_limits(:,2)', ...
                'ztick', REP.hypercube_limits(:,3)');
            axis([min(REP.hypercube_limits(:,1)) max(REP.hypercube_limits(:,1)) ...
                min(REP.hypercube_limits(:,2)) max(REP.hypercube_limits(:,2))]);
        end
        grid on; xlabel('f1'); ylabel('f2'); zlabel('f3');
        drawnow; axis square;
    end
end
% disp(['Generation #0 - Repository size: ' num2str(size(REP.pos,1))]);

%% Main MOPSO Loop
stopCondition = false;
while ~stopCondition
    %% Leader Selection
    % Select a leader from the repository using roulette wheel selection
    h = selectLeader(REP);
    
    %% Velocity and Position Update
    % Update velocities based on inertia, cognitive, and social components
    VEL = W .* VEL + C1 * rand(Np, nVar) .* (PBEST - POS) + ...
        C2 * rand(Np, nVar) .* (repmat(REP.pos(h, :), Np, 1) - POS);
    POS = POS + VEL;
    
    %% Mutation Operation
    % Apply mutation to introduce diversity
    POS = mutation(POS, gen, maxgen, Np, var_max, var_min, nVar, u_mut);
    
    %% Boundary Check
    % Ensure particles remain within search space limits
    [POS, VEL] = checkBoundaries(POS, VEL, maxvel, var_max, var_min);
    
    %% Evaluate New Population
    for i = 1:size(POS, 1)
        vals = num2cell(POS(i, :));   % Convert vector to cell array
        
        [kp, ki, lam, kpi, kii, lami] = deal(vals{:});  % Unpack
        
        OptimizationModel;
        
        % objVal = [IATE_transient, IATE_steady];  % Expected to return a 2x1 vector
        objVal = [rmse_per_time, settling_time];
        % objVal = [overshoot, rmse_per_time];
        % objVal = [overshoot, settling_time];
        
        POS_fit(i, :) = objVal(:)';
    end
    
    %% Repository Update
    % Update repository with new non-dominated solutions
    REP = updateRepository(REP, POS, POS_fit, ngrid);
    if size(REP.pos, 1) > Nr
        REP = deleteFromRepository(REP, size(REP.pos, 1) - Nr, ngrid);
    end
    
    %% Personal Best Update
    % Update personal best positions based on domination criteria
    pos_best = dominates(POS_fit, PBEST_fit);
    best_pos = ~dominates(PBEST_fit, POS_fit);
    best_pos(rand(Np, 1) >= 0.5) = 0;
    if sum(pos_best) > 1
        PBEST_fit(pos_best, :) = POS_fit(pos_best, :);
        PBEST(pos_best, :) = POS(pos_best, :);
    end
    if sum(best_pos) > 1
        PBEST_fit(best_pos, :) = POS_fit(best_pos, :);
        PBEST(best_pos, :) = POS(best_pos, :);
    end
    
    %% Plotting Update (for 2D or 3D problems)
    if plot_figure == true
        if size(POS_fit, 2) == 2
            figure(h_fig); delete(h_par); delete(h_rep);
            h_par = plot(POS_fit(:,1), POS_fit(:,2), 'or'); hold on;
            h_rep = plot(REP.pos_fit(:,1), REP.pos_fit(:,2), 'ok'); hold on;
            try
                set(gca, 'xtick', REP.hypercube_limits(:,1)', 'ytick', REP.hypercube_limits(:,2)');
                axis([min(REP.hypercube_limits(:,1)) max(REP.hypercube_limits(:,1)) ...
                    min(REP.hypercube_limits(:,2)) max(REP.hypercube_limits(:,2))]);
            end
            if isfield(MultiObj, 'truePF')
                try
                    delete(h_pf);
                end
                h_pf = plot(MultiObj.truePF(:,1), MultiObj.truePF(:,2), '.', 'color', 0.8 .* ones(1,3)); hold on;
            end
            grid on; xlabel('f1'); ylabel('f2');
            drawnow; axis square;
        end
        if size(POS_fit, 2) == 3
            figure(h_fig); delete(h_par); delete(h_rep);
            h_par = plot3(POS_fit(:,1), POS_fit(:,2), POS_fit(:,3), 'or'); hold on;
            h_rep = plot3(REP.pos_fit(:,1), REP.pos_fit(:,2), REP.pos_fit(:,3), 'ok'); hold on;
            try
                set(gca, 'xtick', REP.hypercube_limits(:,1)', 'ytick', REP.hypercube_limits(:,2)', ...
                    'ztick', REP.hypercube_limits(:,3)');
                axis([min(REP.hypercube_limits(:,1)) max(REP.hypercube_limits(:,1)) ...
                    min(REP.hypercube_limits(:,2)) max(REP.hypercube_limits(:,2)) ...
                    min(REP.hypercube_limits(:,3)) max(REP.hypercube_limits(:,3))]);
            end
            if isfield(MultiObj, 'truePF')
                try
                    delete(h_pf);
                end
                h_pf = plot3(MultiObj.truePF(:,1), MultiObj.truePF(:,2), MultiObj.truePF(:,3), '.', 'color', 0.8 .* ones(1,3)); hold on;
            end
            grid on; xlabel('f1'); ylabel('f2'); zlabel('f3');
            drawnow; axis square;
        end
    end
    % disp(['Generation #' num2str(gen) ' - Repository size: ' num2str(size(REP.pos,1))]);
    
    %% Generation Update and Termination Check
    gen = gen + 1;
    if gen > maxgen
        stopCondition = true;
    end
end
% hold off;
clear gen;

%% Supporting Functions

% Update the repository with new candidate solutions and refresh the grid
function REP = updateRepository(REP, POS, POS_fit, ngrid)
% Combine new non-dominated solutions with the repository
DOMINATED = checkDomination(POS_fit);
REP.pos     = [REP.pos; POS(~DOMINATED, :)];
REP.pos_fit = [REP.pos_fit; POS_fit(~DOMINATED, :)];
% Remove dominated solutions from the repository
DOMINATED = checkDomination(REP.pos_fit);
REP.pos_fit = REP.pos_fit(~DOMINATED, :);
REP.pos     = REP.pos(~DOMINATED, :);
% Update grid structure based on repository solutions
REP = updateGrid(REP, ngrid);
end

% Ensure particles stay within the defined boundaries and adjust velocities
function [POS, VEL] = checkBoundaries(POS, VEL, maxvel, var_max, var_min)
Np = size(POS, 1);

% Repeat limits for each particle
MAXLIM = repmat(var_max(:)', Np, 1);
MINLIM = repmat(var_min(:)', Np, 1);
MAXVEL = repmat(maxvel(:)', Np, 1);
MINVEL = repmat(-maxvel(:)', Np, 1);

% Cap velocities using elementwise min/max
VEL = min(VEL, MAXVEL);
VEL = max(VEL, MINVEL);

% disp(size(VEL));
% disp(size(MAXVEL));

% Reflect velocity at boundaries and clamp position
VEL(POS > MAXLIM) = -VEL(POS > MAXLIM);
POS(POS > MAXLIM) = MAXLIM(POS > MAXLIM);

VEL(POS < MINLIM) = -VEL(POS < MINLIM);
POS(POS < MINLIM) = MINLIM(POS < MINLIM);
end

% Determine domination status for each particle; returns 1 for dominated particles.
function dom_vector = checkDomination(fitness)
Np = size(fitness, 1);
dom_vector = zeros(Np, 1);
% Generate all pairwise comparisons between particles
all_perm = nchoosek(1:Np, 2);
all_perm = [all_perm; [all_perm(:,2) all_perm(:,1)]];
% Compare pairs to see which particle dominates the other
d = dominates(fitness(all_perm(:,1),:), fitness(all_perm(:,2),:));
dominated_particles = unique(all_perm(d == 1, 2));
dom_vector(dominated_particles) = 1;
end

% Returns a logical vector indicating if each row in x dominates y
function d = dominates(x, y)
d = all(x <= y, 2) & any(x < y, 2);
end

% Update the hypercube grid covering the objective space and assign particles to grids
function REP = updateGrid(REP, ngrid)
ndim = size(REP.pos_fit, 2);
REP.hypercube_limits = zeros(ngrid+1, ndim);
% Determine grid boundaries for each objective dimension
for dim = 1:ndim
    REP.hypercube_limits(:, dim) = linspace(min(REP.pos_fit(:, dim)), max(REP.pos_fit(:, dim)), ngrid+1)';
end

% Assign each particle to a grid (hypercube)
npar = size(REP.pos_fit, 1);
REP.grid_idx = zeros(npar, 1);
REP.grid_subidx = zeros(npar, ndim);
for n = 1:npar
    idnames = [];
    for d = 1:ndim
        REP.grid_subidx(n, d) = find(REP.pos_fit(n, d) <= REP.hypercube_limits(:, d)', 1, 'first') - 1;
        if REP.grid_subidx(n, d) == 0
            REP.grid_subidx(n, d) = 1;
        end
        idnames = [idnames ',' num2str(REP.grid_subidx(n, d))]; %#ok<*AGROW>
    end
    REP.grid_idx(n) = eval(['sub2ind(ngrid * ones(1, ndim)' idnames ');']);
end

% Compute the quality of each hypercube based on particle density
REP.quality = zeros(ngrid, 2);
ids = unique(REP.grid_idx);
for i = 1:length(ids)
    REP.quality(i, 1) = ids(i);                % Hypercube identifier
    REP.quality(i, 2) = 10 / sum(REP.grid_idx == ids(i)); % Quality score
end
end

% Select a leader (global best) using roulette wheel selection based on grid quality
function selected = selectLeader(REP)
% Compute cumulative probability from grid quality values
prob = cumsum(REP.quality(:,2));
sel_hyp = REP.quality(find(rand * max(prob) <= prob, 1, 'first'), 1);

% Choose a random particle from the selected hypercube
idx = 1:length(REP.grid_idx);
selected = idx(REP.grid_idx == sel_hyp);
selected = selected(randi(length(selected)));
end

% Delete extra particles from the repository using crowding distance calculation
function REP = deleteFromRepository(REP, n_extra, ngrid)
% Calculate crowding distances for each repository particle
crowding = zeros(size(REP.pos, 1), 1);
for m = 1:size(REP.pos_fit, 2)
    [m_fit, idx] = sort(REP.pos_fit(:, m), 'ascend');
    m_up   = [m_fit(2:end); Inf];
    m_down = [Inf; m_fit(1:end-1)];
    distance = (m_up - m_down) / (max(m_fit) - min(m_fit));
    [~, idx] = sort(idx, 'ascend');
    crowding = crowding + distance(idx);
end
crowding(isnan(crowding)) = Inf;

% Remove particles with smallest crowding distances
[~, del_idx] = sort(crowding, 'ascend');
del_idx = del_idx(1:n_extra);
REP.pos(del_idx, :) = [];
REP.pos_fit(del_idx, :) = [];
REP = updateGrid(REP, ngrid);
end

% Perform mutation on particles based on current generation
function POS = mutation(POS, gen, maxgen, Np, var_max, var_min, nVar, u_mut)
% Divide the swarm into three segments
fract = Np/3 - floor(Np/3);
if fract < 0.5
    sub_sizes = [ceil(Np/3) round(Np/3) round(Np/3)];
else
    sub_sizes = [round(Np/3) round(Np/3) floor(Np/3)];
end
cum_sizes = cumsum(sub_sizes);

% First segment: no mutation

% Second segment: uniform mutation
nmut = round(u_mut * sub_sizes(2));
if nmut > 0
    idx = cum_sizes(1) + randperm(sub_sizes(2), nmut);
    POS(idx, :) = repmat((var_max - var_min), nmut, 1) .* rand(nmut, nVar) + repmat(var_min, nmut, 1);
end

% Third segment: non-uniform mutation
per_mut = (1 - gen / maxgen)^(5 * nVar); % Mutation probability decreases with generations
nmut = round(per_mut * sub_sizes(3));
if nmut > 0
    idx = cum_sizes(2) + randperm(sub_sizes(3), nmut);
    POS(idx, :) = repmat((var_max - var_min), nmut, 1) .* rand(nmut, nVar) + repmat(var_min, nmut, 1);
end
end