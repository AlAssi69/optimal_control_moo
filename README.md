# PID Tuning using MOO

- Optimal Control HW 01

## By Eng. Ashraf AlAssi @ 19-Dec-25 11:44 PM

```
Project_Root/
├── lib/
│   ├── core/                       % Optimization algorithms (GA, PSO, etc.)
│   └── utils/                      % Helper functions
│       ├── get_system_matrices.m   % Calculates A, B, C, D
│       ├── config.m                % Parameter definitions
│       ├── generate_road_profile.m % Generates road input (Parabolic, etc.)
│       ├── run_simulation.m        % Runs the Simulink model
│       └── calculate_objectives.m  % Calculates RMS costs (J1, J2)
├── models/
│   └── pid_tuning_moo.slx          % Your Simulink model
├── res/                            % Saved .mat files
├── scripts/
│   ├── main_workflow.m             % Main runner
│   └── plot_results.m              % Plotting script
```