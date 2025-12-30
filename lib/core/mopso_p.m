function REP = mopso_p(conf, params, MultiObj, mats, road_ts)
% MOPSO - Multi-Objective Particle Swarm Optimization (PARALLEL VERSION)
%
% This version uses 'parfor' to run Simulink simulations in parallel,
% significantly speeding up the optimization process.

    %% 1. Setup Parallel Pool
    % Check if the parallel pool is running; if not, start it.
    pool_obj = gcp('nocreate');
    if isempty(pool_obj)
        disp('MOPSO: Starting Parallel Pool...');
        parpool;
    end

    %% 2. Unpack Parameters
    Np      = params.Np;            % Population Size
    Nr      = params.Nr;            % Repository Size
    maxgen  = params.maxgen;        % Max Generations
    W       = params.W;             % Inertia Weight
    C1      = params.C1;            % Cognitive Factor
    C2      = params.C2;            % Social Factor
    ngrid   = params.ngrid;         % Grid Divisions
    maxvel  = params.maxvel;        % Max Velocity (%)
    u_mut   = params.u_mut;         % Mutation Rate
    
    nVar    = MultiObj.nVar;        % Number of Variables
    var_min = MultiObj.var_min;     % Lower Bounds
    var_max = MultiObj.var_max;     % Upper Bounds
    
    if nVar ~= 3
        warning('MOPSO:VarCheck', 'Expected 3 variables (Kp, Ki, Kd), but nVar is %d.', nVar);
    end

    %% 3. Initialization
    POS = repmat((var_max - var_min), Np, 1) .* rand(Np, nVar) + repmat(var_min, Np, 1);
    VEL = zeros(Np, nVar);
    
    % We calculate Max Velocity Limit here for use in the loop
    MAX_VEL_LIMIT = (var_max - var_min) .* maxvel ./ 100;

    disp('MOPSO: Initializing population (Parallel Evaluation)...');
    
    %% 4. Evaluate Initial Population (PARALLEL)
    % We use a temporary variable 'costs' because parfor handles sliced outputs better
    costs = zeros(Np, 2); 
    
    parfor i = 1:Np
        % Extract PID Gains for this worker
        % Note: Sliced indexing POS(i, :) is efficient in parfor
        Kp = POS(i, 1);
        Ki = POS(i, 2);
        Kd = POS(i, 3);
        
        try
            % A. Run Simulation
            % Each worker runs its own isolated simulation
            results = run_simulation(conf, mats, road_ts, Kp, Ki, Kd);
            
            % B. Calculate Objectives
            [J1, J2] = calculate_objectives(results);
            
            % Store Costs
            costs(i, :) = [J1, J2];
            
        catch ME
            % Handle unstable particles
            warning('Particle %d unstable: %s', i, ME.message);
            costs(i, :) = [1e9, 1e9]; 
        end
    end
    POS_fit = costs; % Transfer back to main variable
    
    %% 5. Initialize Repository & Personal Bests
    PBEST     = POS;
    PBEST_fit = POS_fit;
    
    DOMINATED = checkDomination(POS_fit);
    REP.pos      = POS(~DOMINATED, :);
    REP.pos_fit  = POS_fit(~DOMINATED, :);
    REP = updateGrid(REP, ngrid);
    
    %% 6. Main MOPSO Loop
    fprintf('MOPSO: Starting Optimization Loop (%d Generations)\n', maxgen);
    
    for gen = 1:maxgen
        
        % --- A. Leader Selection ---
        h = selectLeader(REP);
        
        % --- B. Update Velocity & Position (Vectorized - Fast enough on single core) ---
        VEL = W .* VEL + ...
              C1 * rand(Np, nVar) .* (PBEST - POS) + ...
              C2 * rand(Np, nVar) .* (repmat(REP.pos(h, :), Np, 1) - POS);
          
        POS = POS + VEL;
        
        % --- C. Mutation ---
        POS = mutation(POS, gen, maxgen, Np, var_max, var_min, nVar, u_mut);
        
        % --- D. Boundary Check ---
        [POS, VEL] = checkBoundaries(POS, VEL, MAX_VEL_LIMIT, var_max, var_min);
        
        % --- E. Evaluate New Generation (PARALLEL) ---
        costs = zeros(Np, 2); % Reset costs buffer
        
        parfor i = 1:Np
            Kp = POS(i, 1);
            Ki = POS(i, 2);
            Kd = POS(i, 3);
            
            try
                results = run_simulation(conf, mats, road_ts, Kp, Ki, Kd);
                [J1, J2] = calculate_objectives(results);
                costs(i, :) = [J1, J2];
            catch
                costs(i, :) = [1e9, 1e9];
            end
        end
        POS_fit = costs; % Update main fitness variable
        
        % --- F. Update Repository ---
        REP = updateRepository(REP, POS, POS_fit, ngrid);
        if size(REP.pos, 1) > Nr
            REP = deleteFromRepository(REP, size(REP.pos, 1) - Nr, ngrid);
        end
        
        % --- G. Update Personal Bests ---
        pos_best = dominates(POS_fit, PBEST_fit);      
        best_pos = ~dominates(PBEST_fit, POS_fit);     
        best_pos(rand(Np, 1) >= 0.5) = 0;
        
        if sum(pos_best) > 0
            PBEST_fit(pos_best, :) = POS_fit(pos_best, :);
            PBEST(pos_best, :)     = POS(pos_best, :);
        end
        if sum(best_pos) > 0
            PBEST_fit(best_pos, :) = POS_fit(best_pos, :);
            PBEST(best_pos, :)     = POS(best_pos, :);
        end
        
        % Display Progress
        if mod(gen, 5) == 0 || gen == 1
            fprintf('Gen %d/%d | Rep Size: %d \n', gen, maxgen, size(REP.pos, 1));
        end
    end
    
    disp('MOPSO: Optimization Complete.');
end

%% ========================================================================
%% HELPER FUNCTIONS
%% ========================================================================
% (These remain unchanged, ensure they are included in the file as before)

function REP = updateRepository(REP, POS, POS_fit, ngrid)
    DOMINATED = checkDomination(POS_fit);
    REP.pos     = [REP.pos; POS(~DOMINATED, :)];
    REP.pos_fit = [REP.pos_fit; POS_fit(~DOMINATED, :)];
    DOMINATED = checkDomination(REP.pos_fit);
    REP.pos_fit = REP.pos_fit(~DOMINATED, :);
    REP.pos     = REP.pos(~DOMINATED, :);
    REP = updateGrid(REP, ngrid);
end

function [POS, VEL] = checkBoundaries(POS, VEL, maxvel, var_max, var_min)
    Np = size(POS, 1);
    MAXLIM = repmat(var_max(:)', Np, 1);
    MINLIM = repmat(var_min(:)', Np, 1);
    MAXVEL = repmat(maxvel(:)', Np, 1);
    MINVEL = repmat(-maxvel(:)', Np, 1);
    VEL = min(VEL, MAXVEL);
    VEL = max(VEL, MINVEL);
    VEL(POS > MAXLIM) = -VEL(POS > MAXLIM);
    POS(POS > MAXLIM) = MAXLIM(POS > MAXLIM);
    VEL(POS < MINLIM) = -VEL(POS < MINLIM);
    POS(POS < MINLIM) = MINLIM(POS < MINLIM);
end

function dom_vector = checkDomination(fitness)
    Np = size(fitness, 1);
    dom_vector = zeros(Np, 1);
    all_perm = nchoosek(1:Np, 2);
    all_perm = [all_perm; [all_perm(:,2) all_perm(:,1)]];
    d = dominates(fitness(all_perm(:,1),:), fitness(all_perm(:,2),:));
    dominated_particles = unique(all_perm(d == 1, 2));
    dom_vector(dominated_particles) = 1;
end

function d = dominates(x, y)
    d = all(x <= y, 2) & any(x < y, 2);
end

function REP = updateGrid(REP, ngrid)
    ndim = size(REP.pos_fit, 2);
    REP.hypercube_limits = zeros(ngrid+1, ndim);
    for dim = 1:ndim
        mn = min(REP.pos_fit(:, dim));
        mx = max(REP.pos_fit(:, dim));
        if mx == mn, mx = mn + 1e-6; end
        REP.hypercube_limits(:, dim) = linspace(mn, mx, ngrid+1)';
    end
    npar = size(REP.pos_fit, 1);
    REP.grid_idx = zeros(npar, 1);
    for n = 1:npar
        idnames = [];
        for d = 1:ndim
            idx = find(REP.pos_fit(n, d) <= REP.hypercube_limits(:, d)', 1, 'first') - 1;
            if isempty(idx) || idx == 0, idx = 1; end
            idnames = [idnames ',' num2str(idx)]; %#ok<*AGROW>
        end
        REP.grid_idx(n) = eval(['sub2ind(ngrid * ones(1, ndim)' idnames ');']);
    end
    REP.quality = zeros(ngrid, 2);
    ids = unique(REP.grid_idx);
    for i = 1:length(ids)
        REP.quality(i, 1) = ids(i);
        REP.quality(i, 2) = 10 / sum(REP.grid_idx == ids(i));
    end
end

function selected = selectLeader(REP)
    if isempty(REP.quality)
        selected = randi(size(REP.pos,1));
        return;
    end
    prob = cumsum(REP.quality(:,2));
    sel_hyp = REP.quality(find(rand * max(prob) <= prob, 1, 'first'), 1);
    idx = 1:length(REP.grid_idx);
    candidates = idx(REP.grid_idx == sel_hyp);
    if isempty(candidates)
         selected = randi(size(REP.pos,1));
    else
         selected = candidates(randi(length(candidates)));
    end
end

function REP = deleteFromRepository(REP, n_extra, ngrid)
    crowding = zeros(size(REP.pos, 1), 1);
    for m = 1:size(REP.pos_fit, 2)
        [m_fit, idx] = sort(REP.pos_fit(:, m), 'ascend');
        m_up   = [m_fit(2:end); Inf];
        m_down = [Inf; m_fit(1:end-1)];
        range_val = max(m_fit) - min(m_fit);
        if range_val == 0, range_val = 1e-6; end
        distance = (m_up - m_down) / range_val;
        [~, idx] = sort(idx, 'ascend');
        crowding = crowding + distance(idx);
    end
    crowding(isnan(crowding)) = Inf;
    [~, del_idx] = sort(crowding, 'ascend');
    del_idx = del_idx(1:n_extra);
    REP.pos(del_idx, :) = [];
    REP.pos_fit(del_idx, :) = [];
    REP = updateGrid(REP, ngrid);
end

function POS = mutation(POS, gen, maxgen, Np, var_max, var_min, nVar, u_mut)
    fract = Np/3 - floor(Np/3);
    if fract < 0.5
        sub_sizes = [ceil(Np/3) round(Np/3) round(Np/3)];
    else
        sub_sizes = [round(Np/3) round(Np/3) floor(Np/3)];
    end
    cum_sizes = cumsum(sub_sizes);
    nmut = round(u_mut * sub_sizes(2));
    if nmut > 0
        idx = cum_sizes(1) + randperm(sub_sizes(2), nmut);
        POS(idx, :) = repmat((var_max - var_min), nmut, 1) .* rand(nmut, nVar) + repmat(var_min, nmut, 1);
    end
    per_mut = (1 - gen / maxgen)^(5 * nVar);
    nmut = round(per_mut * sub_sizes(3));
    if nmut > 0
        idx = cum_sizes(2) + randperm(sub_sizes(3), nmut);
        POS(idx, :) = repmat((var_max - var_min), nmut, 1) .* rand(nmut, nVar) + repmat(var_min, nmut, 1);
    end
end