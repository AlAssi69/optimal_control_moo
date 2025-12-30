function [J1, J2] = calculate_objectives(sim_results)
% CALCULATE_OBJECTIVES - Returns Cost Values for MOO using ITAE
%
% J1: Comfort Objective (ITAE of Sprung Acceleration)
%     Target: Minimize impact on passenger and ensure vibrations die out fast.
%
% J2: Handling & Stability (ITAE of Suspension & Tire Deflection)
%     Target: Keep tire on road (safety) and return suspension to center (travel).

% Extract signals
t = sim_results.time;
acc = sim_results.sprung_acc;       % Sprung Mass Acceleration
sus_def = sim_results.sus_def;      % Suspension Deflection (x1)
tire_def = sim_results.tire_def;    % Tire Deflection (x3)

% --- Helper: ITAE Calculation ---
% Formula: Integral from 0 to T of (t * |error|) dt
% We use trapz() for numerical integration
calc_itae = @(signal) trapz(t, t .* abs(signal));

% ---------------------------------------------------------
% OBJECTIVE 1: COMFORT
% ---------------------------------------------------------
% We want the passenger to feel nothing (acc = 0) and for any
% bump feeling to vanish immediately (Time weighting).
J1 = calc_itae(acc);

% ---------------------------------------------------------
% OBJECTIVE 2: HANDLING & ROAD HOLDING
% ---------------------------------------------------------
% 1. Suspension Deflection: Must settle to 0 to be ready for next bump.
% 2. Tire Deflection: Must be minimized to ensure grip (tire on road).

itae_sus = calc_itae(sus_def);
itae_tire = calc_itae(tire_def);

% We combine them into a single "Stability" cost.
% Note: You can add weights (w1, w2) if you want to prioritize one.
% For now, equal weighting is a good starting point.
J2 = itae_sus + itae_tire;

% ---------------------------------------------------------
% OPTIONAL: Physical Constraint Penalty
% ---------------------------------------------------------
% If the suspension hits the bump stops (> 15cm), add a huge penalty.
% This tells the optimizer: "This solution is INVALID."
limit_m = 0.15;
if max(abs(sus_def)) > limit_m
    J2 = J2 + 1e6; % Huge penalty to reject this solution
end
end