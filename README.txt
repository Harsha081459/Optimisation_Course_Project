Minimum-Energy Trajectory Control for Autonomous Lane Change
TEAM OPTICORS - Optimisation Course Project

Course: Optimization

TEAM MEMBERS:
- D Harsha Vardhan (BT2024108)
- P Prajwal (BT2024245)


PROJECT DESCRIPTION:
This project implements an optimal trajectory generation system for an
autonomous vehicle performing a lane change maneuver while avoiding a dynamic obstacle.

The vehicle is modeled using discrete-time double-integrator dynamics, and the problem
is formulated as a Constrained Quadratic Program (QP). The goal is to minimize total control
energy while satisfying safety constraints on motion and obstacle avoidance.


KEY FEATURES:
- QP formulation with linear equality and inequality constraints
- Vehicle dynamics: x_{k+1} = A x_k + B u_k
- Obstacle avoidance using time-window constraints
- KKT sensitivity analysis (Lagrange multipliers / shadow prices)
- Solver comparison between OSQP and SCS
- Interactive user inputs for simulation parameters


FILE STRUCTURE:
OPTICORS/
├── Implementation.ipynb          (Main implementation and visualizations)
├── Project_Report.pdf           (Detailed exploration results and methodology)
└── README.md                    (Documentation)


SETUP & EXECUTION:
1. Extract the OPTICORS.zip folder
2. Open a terminal in the extracted directory
3. Launch Jupyter Notebook: jupyter notebook
4. Open Implementation.ipynb
5. Run all cells sequentially


INTERACTIVE INPUTS:
During execution, you will be prompted for simulation parameters. 
Press ENTER to accept defaults.


Default parameters:

Input               Description                           Default

dt                 Time step (seconds)                    0.1
N                  Number of simulation steps             50    
init_y             Initial lateral position (m)           0.0   
init_vy            Initial lateral velocity (m/s)         0.0   
target_y           Target lane center (m)                 3.5   
final_vy_target    Required final lateral velocity (m/s)| 0.0   
u_max              Maximum lateral acceleration (m/s²)    3.0   
v_max              Maximum lateral velocity (m/s)         2.0   
jerk_max           Soft-constraint penalty weight         1e6   
plot_result        Plot trajectories? (y/n)              y    
Obstacle Start     Start time of obstacle window (s)      2.5   
Obstacle End       Maximum jerk (m/s³)                    2.0   
penalty_slack      End time of obstacle window (s)        4.5



EXPECTED OUTPUTS:

1. Solver Logs:
   - Optimality status
   - Runtime comparison (OSQP vs SCS)

2. Plots & Visualizations:
   - Runtime comparison bar chart
   - KKT condition verification (primal & dual feasibility)
   - Trajectory plot showing lane change and real obstacle zone
   - Lagrange multiplier bar plot indicating obstacle sensitivity


PREREQUISITES & LIBRARIES:
The project uses Python 3.

Required Libraries:
- numpy
- cvxpy
- matplotlib
- time (built-in)

Installation:
pip install numpy cvxpy matplotlib

SOLVERS USED:
- OSQP (primary QP solver due to speed & robustness)
- SCS (for benchmarking performance)