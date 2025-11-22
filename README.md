# Minimum-Energy Trajectory Control for Autonomous Lane Change

**TEAM OPTICORS** - Optimisation Course Project

**Course**: Optimization  
**Submission Date**: 23/11/2025


## 👥 Team Members

- **D Harsha Vardhan** (BT2024108)
- **P Prajwal** (BT2024245)

---

## 📋 Project Description

This project implements an optimal trajectory generation system for an autonomous vehicle performing a lane change maneuver while avoiding a dynamic obstacle.

The vehicle is modeled using discrete-time double-integrator dynamics, and the problem is formulated as a Constrained Quadratic Program (QP). The goal is to minimize total control energy while satisfying safety constraints on motion and obstacle avoidance.

### Key Features

- **QP formulation** with linear equality and inequality constraints
- **Vehicle dynamics**:
  \[ 
  x_{k+1} = A x_k + B u_k 
  \]
- **Obstacle avoidance** using time-window constraints
- **KKT sensitivity analysis** (Lagrange multipliers / shadow prices)
- **Solver comparison** between OSQP and SCS
- **Interactive user inputs** for simulation parameters

---

## 2. FILE STRUCTURE
The project directory contains the following key files:

* Implementation.ipynb : *(MAIN)* The Jupyter Notebook containing the code, visualizations, and analysis.
* Implementation.py : The standalone Python script version of the code.
* Report.pdf : The detailed project report covering formulation, methodology, and results.
* 1pageproposal.pdf : The original project proposal document.
* README.md : This documentation file.


---

## 🚀 Setup & Execution

1. Extract the `OPTICORS.zip` folder
2. Open a terminal in the extracted directory
3. Launch Jupyter Notebook:
   ```bash
   jupyter notebook

Run all cells sequentially
Interactive Inputs
During execution, you will be prompted for simulation parameters. (Press ENTER to accept defaults)

Default parameters:


| Input               | Description                           | Default |
| ------------------- | ------------------------------------- | ------- |
| **dt**              | Time step (seconds)                   | `0.1`   |
| **N**               | Number of simulation steps            | `50`    |
| **init_y**          | Initial lateral position (m)          | `0.0`   |
| **init_vy**         | Initial lateral velocity (m/s)        | `0.0`   |
| **target_y**        | Target lane center (m)                | `3.5`   |
| **final_vy_target** | Required final lateral velocity (m/s) | `0.0`   |
| **u_max**           | Maximum lateral acceleration (m/s²)   | `3.0`   |
| **v_max**           | Maximum lateral velocity (m/s)        | `2.0`   |
| **jerk_max**        | Maximum jerk (m/s³)                   | `2.0`   |
| **penalty_slack**   | Soft-constraint penalty weight        | `1e6`   |
| **plot_result**     | Plot trajectories? (`y/n`)            | `y`     |
| **Obstacle Start**  | Start time of obstacle window (s)     | `2.5`   |
| **Obstacle End**    | End time of obstacle window (s)       | `4.5`   |




---



## 📊 Expected Outputs

1. **Solver Logs**
       
    1. Optimality status
    2. Runtime comparison between OSQP and SCS


2. **Plots & Visualizations**

    1. Solver runtime bar chart
    2. KKT condition verification (primal & dual feasibility)
    3. Trajectory plot showing lane change and obstacle zone (red region)
    4. Lagrange multipliers plot showing obstacle constraint sensitivity


---


## 🛠 Prerequisites & Libraries

The project uses **Python 3**.

### Required Libraries

- `numpy`
- `cvxpy`
- `matplotlib`
- `time` (built-in)

### Installation

```bash
pip install numpy cvxpy matplotlib

Solvers Used
OSQP (primary QP solver due to speed & robustness)

SCS (for benchmarking performance)
