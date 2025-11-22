TEAM OPTICORE - OPTIMIZATION COURSE PROJECT


Project Title:    Minimum-Energy Trajectory Control (Autonomous Lane Change)
Course:           Optimization
Submission Date:  23/11/2025


Team Members:
1. D Harsha Vardhan (BT2024106)
2. P Prajwal        (BT2024245)


1. PROJECT DESCRIPTION
    This project solves the optimal trajectory generation problem for an autonomous 
    vehicle performing a lane-change maneuver while avoiding a dynamic obstacle. 
    
    We model the vehicle using discrete-time double-integrator dynamics and formulate
    the control problem as a Constrained Convex Quadratic Program (QP). The objective 
    is to minimize the control effort (energy) required to move the vehicle from its 
    initial lane to a target lane within a fixed time horizon, subject to safety 
    constraints on velocity, acceleration, jerk, and obstacle avoidance.

    Key Features:
    - Mathematical Formulation: Convex QP with linear equality/inequality constraints.
    - Dynamics Model: Discrete-time state-space model ($x_{k+1} = Ax_k + Bu_k$).
    - Obstacle Avoidance: Time-window constraints forcing a safe lane merge.
    - Analysis: KKT/Lagrangian analysis to evaluate constraint sensitivity (Shadow Prices).
    - Solver Comparison: Performance benchmarking between OSQP and SCS solvers.

  
2. FILE STRUCTURE

    The submitted zip file contains:
    
    1. 'Implementation.ipynb'  : The main Jupyter Notebook containing all code, 
                                 visualizations, and analysis.
    2. 'README.txt'            : This file (Project documentation and setup guide).
    3. 'Project_Report.pdf'    : Detailed report covering formulation, methodology, 
                                 and results.


3. PREREQUISITES & LIBRARIES

    The project is implemented in Python 3. The following libraries are required 
    to run the code:
    
    - numpy       (for numerical operations and matrix manipulation)
    - cvxpy       (for defining and solving the convex optimization problem)
    - matplotlib  (for plotting trajectories and KKT analysis graphs)
    - time        (standard library, for solver runtime comparison)
    
    To install dependencies, run:
    $ pip install numpy cvxpy matplotlib


4. SETUP & EXECUTION INSTRUCTIONS

    1. Unzip the folder 'Opticore.zip'.
    2. Open a terminal or command prompt in the extracted folder.
    3. Launch Jupyter Notebook:
       $ jupyter notebook
    4. Open the file 'Implementation.ipynb'.
    5. Run all cells sequentially.

    -- Interactive Inputs --
    When running the main code cell, you will be prompted to enter simulation 
    parameters. You can press ENTER to accept the default values (recommended for 
    first run) or type your own.
    
    Default Parameters:
    - Time step (dt): 0.1 s
    - Horizon (N): 50 steps
    - Obstacle Start: 2.5 s
    - Obstacle End:   4.5 s


5. EXPECTED OUTPUTS

    Upon successful execution, the notebook will generate:
    
    1. Solver Logs:
       - Status (Optimal/Infeasible)
       - Runtime comparison between OSQP and SCS solvers.
    
    2. Visualizations:
       - "Solver Runtime Comparison": Bar chart of execution times.
       - "OSQP: Dual/Primal Feasibility": Stem plots validating KKT conditions.
       - "Trajectory Comparison": 2D plot showing the vehicle's path swerving 
         to avoid the red "Obstacle Zone".
       - "Obstacle Sensitivity Analysis": A bar graph of Lagrange Multipliers 
         showing the "cost" imposed by the obstacle constraint at each time step.


6. METHODOLOGY & SOLVER DETAILS
    
    - Formulation: We formulated the problem as minimizing $\sum u_k^2$ subject to 
      linear dynamics and box constraints. Slack variables were added to soft 
      constraints to ensure feasibility.
    - Tool: CVXPY was used as the modeling language.
    - Solvers:
      - OSQP (Operator Splitting Quadratic Program): Selected as the primary solver 
        due to its speed and robustness for QPs.
      - SCS (Splitting Conic Solver): Used for benchmarking purposes.

===============================================================================
