function plot_pareto_front(REP, best_costs)
% PLOT_PARETO_FRONT - Visualizes the MOPSO Trade-off Curve
%
% Inputs:
%   REP:        Repository struct from MOPSO (contains .pos_fit)
%   best_costs: 1x2 vector of the selected solution's costs [J1, J2]

% --- 1. Create Figure with White Background ---
fig = figure('Name', 'Pareto Front', 'Color', 'w', 'Position', [150, 100, 800, 600]);

% --- LIGHT MODE OVERRIDE ---------------------------------------------
set(fig, 'DefaultAxesColor', 'w');
set(fig, 'DefaultAxesXColor', 'k');
set(fig, 'DefaultAxesYColor', 'k');
set(fig, 'DefaultAxesZColor', 'k');
set(fig, 'DefaultTextColor', 'k');
set(fig, 'DefaultLegendTextColor', 'k');
set(fig, 'DefaultAxesGridColor', [0.15 0.15 0.15]);
set(fig, 'DefaultAxesGridAlpha', 0.15);
% ---------------------------------------------------------------------

% --- 2. Plot the Data ---
% Plot all non-dominated solutions in Red
plot(REP.pos_fit(:,1), REP.pos_fit(:,2), 'ro', ...
     'MarkerFaceColor', 'r', 'MarkerSize', 6);
hold on;

% Plot the selected "Best" solution in Blue Square
plot(best_costs(1), best_costs(2), 'bs', ...
     'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 1.5);

% --- 3. Labels and Formatting ---
xlabel('Comfort Objective (ITAE Acceleration)');
ylabel('Handling Objective (ITAE Deflection)');
title('Pareto Front: Trade-off between Comfort and Handling', 'Color', 'k');
grid on;

% --- 4. LEGEND (Explicit Color Locking) ---
lgd = legend('Non-Dominated Solutions', 'Selected Balanced Solution', ...
     'Location', 'best');

% Force Legend properties to ensure visibility against white background
set(lgd, 'TextColor', 'k');      % Black Text
set(lgd, 'Color', 'w');          % White Background
set(lgd, 'EdgeColor', 'k');      % Black Border

% --- 5. Axes Adjustment ---
axis tight;
ax = gca;
% Add 10% padding so points don't sit exactly on the frame
ax.XLim = [min(ax.XLim)*0.9, max(ax.XLim)*1.1];
ax.YLim = [min(ax.YLim)*0.9, max(ax.YLim)*1.1];
end