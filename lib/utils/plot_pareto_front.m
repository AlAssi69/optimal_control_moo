function plot_pareto_front(REP, points)
% PLOT_PARETO_FRONT - Visualizes the MOPSO Trade-off Curve with 3 Key Points
%
% Inputs:
%   REP:    Repository struct from MOPSO (contains .pos_fit)
%   points: Struct containing .balanced, .comfort, and .handling fields
%           Each field must have a .costs vector [J1, J2]

% --- 1. Create Figure with White Background ---
fig = figure('Name', 'Pareto Front', 'Color', 'w', 'Position', [150, 100, 800, 600]);

% --- LIGHT MODE OVERRIDE ---------------------------------------------
set(fig, 'DefaultAxesColor', 'w');
set(fig, 'DefaultAxesXColor', 'k');
set(fig, 'DefaultAxesYColor', 'k');
set(fig, 'DefaultTextColor', 'k');
set(fig, 'DefaultLegendTextColor', 'k');
set(fig, 'DefaultAxesGridColor', [0.15 0.15 0.15]);
set(fig, 'DefaultAxesGridAlpha', 0.15);
% ---------------------------------------------------------------------

% --- 2. Plot All Non-Dominated Solutions ---
% Plot these first (Gray/Red circles)
h_all = plot(REP.pos_fit(:,1), REP.pos_fit(:,2), 'o', ...
     'MarkerEdgeColor', [0.5 0.5 0.5], ... % Grey Edge
     'MarkerFaceColor', [0.9 0.9 0.9], ... % Light Grey Face
     'MarkerSize', 6);
hold on;

% --- 3. Plot The Three Special Points ---

% A. Balanced (Blue Square)
h_bal = plot(points.balanced.costs(1), points.balanced.costs(2), 's', ...
     'MarkerSize', 12, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', ...
     'LineWidth', 1.5);

% B. Best Comfort (Green Triangle)
h_comf = plot(points.comfort.costs(1), points.comfort.costs(2), '^', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', ...
     'LineWidth', 1.5);

% C. Best Handling (Magenta Diamond)
h_hand = plot(points.handling.costs(1), points.handling.costs(2), 'd', ...
     'MarkerSize', 10, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', ...
     'LineWidth', 1.5);

% --- 4. Labels and Formatting ---
xlabel('Comfort Objective (ITAE Acceleration)');
ylabel('Handling Objective (ITAE Deflection)');
title('Pareto Front: Trade-off between Comfort and Handling', 'Color', 'k');
grid on;

% --- 5. Legend ---
lgd = legend([h_all, h_bal, h_comf, h_hand], ...
     'Pareto Front', ...
     'Balanced Solution', ...
     'Best Comfort (Min Accel)', ...
     'Best Handling (Min Deflect)', ...
     'Location', 'best');

% Force Legend Visibility
set(lgd, 'TextColor', 'k');
set(lgd, 'Color', 'w');
set(lgd, 'EdgeColor', 'k');

% --- 6. Axes Adjustment ---
axis tight;
ax = gca;
% Add padding
x_rng = max(ax.XLim) - min(ax.XLim);
y_rng = max(ax.YLim) - min(ax.YLim);
ax.XLim = [min(ax.XLim) - 0.05*x_rng, max(ax.XLim) + 0.05*x_rng];
ax.YLim = [min(ax.YLim) - 0.05*y_rng, max(ax.YLim) + 0.05*y_rng];
end