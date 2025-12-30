# PID Tuning using Multi-Objective Optimization (MOO)

**Optimal Control HW 01**  
**By Eng. Ashraf AlAssi @ 19-Dec-25 11:44 PM**

This project implements a Multi-Objective Particle Swarm Optimization (MOPSO) algorithm to tune PID controller gains for a quarter-car suspension system. The optimization balances two competing objectives: passenger comfort (minimizing acceleration) and vehicle handling (minimizing suspension and tire deflection).

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Project Structure](#project-structure)
3. [File Descriptions](#file-descriptions)
4. [How to Run the Project](#how-to-run-the-project)
5. [Parameter Configuration](#parameter-configuration)
6. [Project Tree](#project-tree)

---

## Project Overview

The system optimizes PID controller parameters (Kp, Ki, Kd) for an active suspension system using:
- **Objective 1 (J1)**: Comfort - ITAE of sprung mass acceleration
- **Objective 2 (J2)**: Handling - ITAE of suspension and tire deflection

The MOPSO algorithm finds a Pareto front of optimal solutions, allowing you to choose between different trade-offs.

---

## Project Structure

```
MOO/
├── lib/                          % Core library functions
│   ├── core/                     % Optimization algorithms
│   │   ├── mopso_p.m            % MOPSO (Parallel version - recommended)
│   │   └── mopso_s.m            % MOPSO (Sequential version)
│   └── utils/                    % Helper functions
│       ├── config.m              % System parameter definitions
│       ├── get_system_matrices.m % State-space matrices (A, B, C, D)
│       ├── generate_road_profile.m % Road input generation
│       ├── run_simulation.m      % Simulink model execution
│       ├── calculate_objectives.m % Objective function calculation
│       ├── plot_pareto_front.m   % Pareto front visualization
│       └── plot_simulation_results.m % Simulation results plotting
├── models/                       % Simulink models
│   └── pid_tuning_moo.mdl       % Quarter-car suspension model
├── scripts/                      % Main execution scripts
│   ├── main_workflow.m          % Main optimization workflow
│   └── load_and_plot_results.m  % Load and visualize saved results
├── res/                          % Results storage
│   ├── baseline_results.mat     % Baseline simulation results
│   └── optimization_results.mat  % MOPSO optimization results
└── README.md                     % This file
```

---

## File Descriptions

### Core Optimization Algorithms

#### `lib/core/mopso_p.m`
**Purpose**: Multi-Objective Particle Swarm Optimization (Parallel Version)  
**Description**: Implements MOPSO using MATLAB's parallel computing toolbox (`parfor`) to evaluate multiple particles simultaneously. This significantly speeds up optimization by running Simulink simulations in parallel.  
**Key Features**:
- Parallel evaluation of population fitness
- Grid-based archive management
- Leader selection from repository
- Mutation operators for diversity

#### `lib/core/mopso_s.m`
**Purpose**: Multi-Objective Particle Swarm Optimization (Sequential Version)  
**Description**: Standard sequential implementation of MOPSO. Use this if parallel computing is unavailable or if you encounter issues with `mopso_p.m`.  
**Key Features**: Same algorithm as parallel version, but evaluates particles sequentially.

### Utility Functions

#### `lib/utils/config.m`
**Purpose**: Configuration file containing all system parameters  
**Description**: Returns a struct with vehicle parameters, simulation settings, road profile settings, and default PID gains. This is the central configuration file for the entire project.

#### `lib/utils/get_system_matrices.m`
**Purpose**: Generates state-space matrices for the quarter-car model  
**Description**: Calculates the A, B, C, D matrices of the state-space representation based on vehicle parameters from `config.m`.  
**Output**: State-space system object and raw matrices struct.

#### `lib/utils/generate_road_profile.m`
**Purpose**: Creates road disturbance input for simulation  
**Description**: Generates road profile (height and velocity) based on configuration. Supports two road types:
- `'parabolic'`: Parabolic bump profile
- `'haversine'`: Haversine (smooth) bump profile  
**Output**: Timeseries object for Simulink and road profile vectors.

#### `lib/utils/run_simulation.m`
**Purpose**: Executes the Simulink model with given PID gains  
**Description**: Configures and runs the Simulink model `pid_tuning_moo.mdl` with specified controller gains. Injects system matrices, road profile, and PID parameters into the model workspace.  
**Output**: Standardized results struct containing time histories of all signals.

#### `lib/utils/calculate_objectives.m`
**Purpose**: Computes objective function values (J1, J2)  
**Description**: Calculates ITAE-based costs:
- **J1**: ITAE of sprung mass acceleration (comfort)
- **J2**: ITAE of suspension deflection + tire deflection (handling)  
**Note**: Applies penalty if suspension deflection exceeds physical limits.

#### `lib/utils/plot_pareto_front.m`
**Purpose**: Visualizes the Pareto front and key solutions  
**Description**: Creates a scatter plot showing all non-dominated solutions and highlights three special points:
- Balanced solution (closest to origin)
- Best comfort solution (minimum J1)
- Best handling solution (minimum J2)

#### `lib/utils/plot_simulation_results.m`
**Purpose**: Plots comprehensive simulation results  
**Description**: Creates a 6-panel figure showing:
1. Road profile height
2. Road velocity input
3. Suspension deflection
4. Tire deflection
5. Sprung mass acceleration (with RMS)
6. Control force

### Main Scripts

#### `scripts/main_workflow.m`
**Purpose**: Main execution script for the optimization workflow  
**Description**: Orchestrates the entire optimization process:
1. Loads configuration
2. Calculates system matrices
3. Generates road profile
4. Configures MOPSO parameters
5. Runs optimization
6. Selects key solutions (balanced, comfort, handling)
7. Validates and plots results
8. Saves optimization results

**Usage**: Run this script to perform the complete optimization.

#### `scripts/load_and_plot_results.m`
**Purpose**: Reloads and visualizes saved optimization results  
**Description**: Loads previously saved results from `res/optimization_results.mat` and regenerates all plots without re-running the optimization. Useful for post-processing and analysis.

### Models

#### `models/pid_tuning_moo.mdl`
**Purpose**: Simulink model of the quarter-car suspension system  
**Description**: Implements the quarter-car model with active suspension control. The model uses state-space representation and includes a PID controller. Make sure this model is properly configured with signal logging enabled for:
- Suspension Deflection
- Sprung Acceleration
- Tire Deflection
- Force (optional)

---

## How to Run the Project

### Prerequisites

1. **MATLAB** (R2018b or later recommended)
2. **Simulink** toolbox
3. **Parallel Computing Toolbox** (optional, but recommended for faster optimization)
4. Simulink model: `models/pid_tuning_moo.mdl` must be available

### Step-by-Step Instructions

#### Option 1: Full Optimization Workflow

1. **Open MATLAB** and navigate to the `MOO/scripts/` directory:
   ```matlab
   cd('MOO/scripts')
   ```

2. **Run the main workflow**:
   ```matlab
   main_workflow
   ```

3. **Wait for completion**: The optimization will:
   - Display progress messages
   - Show generation updates every 5 iterations
   - Generate plots automatically
   - Save results to `res/optimization_results.mat`

4. **Review results**: Three figures will be generated:
   - Pareto Front plot
   - Balanced solution results
   - Best comfort solution results
   - Best handling solution results

#### Option 2: Load and Plot Existing Results

If you have already run the optimization and want to regenerate plots:

1. Navigate to the scripts directory:
   ```matlab
   cd('MOO/scripts')
   ```

2. Run the load script:
   ```matlab
   load_and_plot_results
   ```

### Expected Runtime

- **Sequential version** (`mopso_s`): ~30-60 minutes (depending on `maxgen` and `Np`)
- **Parallel version** (`mopso_p`): ~10-20 minutes (with 4-8 workers)

### Troubleshooting

- **"mopso_p not found"**: The script will automatically fall back to `mopso_s.m`
- **Parallel pool errors**: Ensure Parallel Computing Toolbox is installed and licensed
- **Simulink errors**: Verify that `pid_tuning_moo.mdl` exists and signal logging is enabled
- **Path errors**: Make sure you're running from the `scripts/` directory

---

## Parameter Configuration

### 1. System Parameters (`lib/utils/config.m`)

Edit `config.m` to modify vehicle and simulation parameters:

```matlab
% Vehicle Parameters
conf.ms = 250;      % Sprung mass (kg)
conf.mu = 45;       % Unsprung mass (kg)
conf.ks = 28000;    % Suspension stiffness (N/m)
conf.bs = 2500;     % Suspension damping (N·s/m)
conf.kt = 190000;   % Tire stiffness (N/m)

% Simulation Settings
conf.T_end = 5;         % Total simulation time (s)
conf.dt = 0.001;        % Time step (s)

% Road Profile Settings
conf.road_type = 'haversine'; % 'parabolic' or 'haversine'
conf.bump_height = 0.15;      % Bump height (m)
conf.bump_length = 5.0;       % Bump length (m)
conf.car_speed_kmh = 30;      % Vehicle speed (km/h)
conf.start_time = 1.0;        % Bump start time (s)
```

**Guidelines**:
- **Road type**: `'haversine'` provides smoother transitions; `'parabolic'` is more abrupt
- **Bump height**: Typical values: 0.1-0.2 m
- **Simulation time**: Should be long enough for system to settle (typically 3-5 seconds)

### 2. Optimization Parameters (`scripts/main_workflow.m`)

Edit the MOPSO parameters in `main_workflow.m`:

```matlab
% Solver Parameters
params.Np = 50;          % Population Size
params.Nr = 100;         % Repository/Archive Size
params.maxgen = 50;      % Number of Generations
params.W = 0.4;          % Inertia Weight
params.C1 = 2.0;         % Cognitive Factor
params.C2 = 2.0;         % Social Factor
params.ngrid = 20;       % Grid Divisions
params.maxvel = 5;       % Max Velocity (%)
params.u_mut = 0.5;      % Mutation Rate

% Search Space Bounds
MultiObj.var_min = [1000, 0,   100];   % [Kp_min, Ki_min, Kd_min]
MultiObj.var_max = [50000, 500, 5000];  % [Kp_max, Ki_max, Kd_max]
```

**Parameter Guidelines**:

| Parameter | Description | Recommended Range | Notes |
|-----------|-------------|-------------------|-------|
| `Np` | Population size | 30-100 | Higher = better exploration, slower |
| `maxgen` | Generations | 30-100 | Higher = better convergence |
| `Nr` | Repository size | 50-200 | Stores non-dominated solutions |
| `W` | Inertia weight | 0.3-0.5 | Controls exploration vs exploitation |
| `C1`, `C2` | Learning factors | 1.5-2.5 | Balance personal vs social learning |
| `ngrid` | Grid divisions | 10-30 | Affects leader selection diversity |
| `maxvel` | Max velocity (%) | 3-10 | Limits particle movement speed |
| `u_mut` | Mutation rate | 0.3-0.7 | Maintains population diversity |

**Search Space Guidelines**:
- **Kp (Proportional)**: Typically 1000-50000
- **Ki (Integral)**: Typically 0-500 (start with 0 if unsure)
- **Kd (Derivative)**: Typically 100-5000

**Tips**:
- Start with smaller `Np` (30-50) and `maxgen` (30-50) for quick tests
- Increase for final runs to get better Pareto fronts
- Adjust bounds based on your system's physical constraints
- If optimization is slow, reduce `Np` or `maxgen`
- If results are poor, expand the search space bounds

### 3. Objective Function Weights (`lib/utils/calculate_objectives.m`)

Currently, J2 combines suspension and tire deflection with equal weights. To prioritize one over the other, modify line 40:

```matlab
% Current (equal weights):
J2 = itae_sus + itae_tire;

% Example with weights:
w1 = 0.7;  % Weight for suspension deflection
w2 = 0.3;  % Weight for tire deflection
J2 = w1 * itae_sus + w2 * itae_tire;
```

---

## Project Tree

```
MOO/
│
├── lib/                                    % Core library
│   ├── core/                               % Optimization algorithms
│   │   ├── mopso_p.m                      % MOPSO parallel version
│   │   └── mopso_s.m                      % MOPSO sequential version
│   │
│   └── utils/                              % Utility functions
│       ├── config.m                        % System configuration
│       ├── get_system_matrices.m          % State-space matrices
│       ├── generate_road_profile.m        % Road profile generator
│       ├── run_simulation.m               % Simulink runner
│       ├── calculate_objectives.m         % Objective calculator
│       ├── plot_pareto_front.m           % Pareto front plotter
│       └── plot_simulation_results.m     % Results plotter
│
├── models/                                 % Simulink models
│   └── pid_tuning_moo.mdl                 % Quarter-car model
│
├── scripts/                                % Execution scripts
│   ├── main_workflow.m                    % Main optimization script
│   ├── load_and_plot_results.m           % Results loader
│   └── slprj/                             % Simulink cache (auto-generated)
│
├── res/                                    % Results storage
│   ├── baseline_results.mat              % Baseline simulation
│   └── optimization_results.mat          % MOPSO results
│
├── LICENSE                                 % License file
└── README.md                               % This documentation
```

---

## Additional Notes

- **Parallel Computing**: The parallel version (`mopso_p.m`) automatically starts a parallel pool if one doesn't exist. You can manually control this with `parpool` and `delete(gcp('nocreate'))`.
- **Results Storage**: All optimization results are saved to `res/optimization_results.mat` and can be reloaded later without re-running optimization.
- **Model Requirements**: Ensure your Simulink model has signal logging enabled for all required signals (see `run_simulation.m` for signal names).
- **Performance**: For faster optimization, use `mopso_p.m` with a parallel pool of 4-8 workers.

---

## Contact

For questions or issues, contact me.

---

**Last Updated**: 30-Dec-25