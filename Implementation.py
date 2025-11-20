# importing libraries

import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

# input helper function
def get_input(prompt, default, cast=float, validator=None):
    while True:
        s = input(f"{prompt} [{default}]: ").strip()
        if s == "":
            return default
        try:
            val = cast(s)
            if validator is None or validator(val):
                return val
            print("Validation failed. Try again.")
        except:
            print("Invalid input. Try again.")
# ------- main --------
def main():
    print("v10.py — v9 plus plotting (no obstacle shown)")
    dt = get_input("dt (s)", 0.1, float, lambda v: v>0)
    N = get_input("N (steps, integer >=1)", 50, int, lambda v: v>=1)
    init_y = get_input("init_y (m)", 0.0, float)
    init_vy = get_input("init_vy (m/s)", 0.0, float)
    target_y = get_input("target_y (m)", 3.5, float)
    final_vy_target = get_input("final_vy_target (m/s)", 0.0, float)
    u_max = get_input("u_max (m/s^2)", 3.0, float, lambda v: v>=0)
    v_max = get_input("v_max (m/s)", 2.0, float, lambda v: v>=0)
    jerk_max = get_input("jerk_max (m/s^3)", 2.0, float, lambda v: v>=0)
    penalty_slack = get_input("penalty_slack", 1e6, float, lambda v: v>0)
    plot_result = input("plot_result (y/n) [y]: ").strip().lower()
    if plot_result == "": plot_result = "y"

    # dynamic matrices
    A = np.array([[1.0, dt],[0.0, 1.0]])
    B = np.array([[0.5*dt*dt],[dt]])

    # Decision variables
    u = cp.Variable((N,1))
    x = cp.Variable((2, N+1))
    slack_u = cp.Variable((N,1))
    slack_v = cp.Variable((N+1,1))
    slack_final = cp.Variable((1,1))

    constraints = [x[:,0] == np.array([init_y, init_vy], dtype=float)]
    cost = 0
    for k in range(N):
        constraints += [x[:,k+1] == A @ x[:,k] + B[:,0]*u[k,0]]
        constraints += [cp.abs(u[k,0]) <= u_max + slack_u[k,0]]
        if k > 0:
            constraints += [cp.abs(u[k,0] - u[k-1,0]) <= jerk_max]
        cost += cp.sum_squares(u[k,0])
    for k in range(N+1):
        constraints += [cp.abs(x[1,k]) <= v_max + slack_v[k,0]]

    constraints += [x[1,N] == final_vy_target]
    constraints += [x[0,N] == target_y + slack_final[0,0]]
    constraints += [slack_u >= 0, slack_v >= 0, slack_final >= 0]
    cost += penalty_slack * (cp.sum_squares(slack_u) + cp.sum_squares(slack_v) + cp.sum_squares(slack_final))

    # Realistic obstacle timing window (time-mapped indices)
    # Default window ≈ 2.5s to 4.5s after start
    k_obs_start = int(2.5 / dt)
    k_obs_end = int(4.5 / dt)
    k_obs_start = max(0, min(N-1, k_obs_start))
    k_obs_end = max(k_obs_start+1, min(N, k_obs_end))
    for k in range(k_obs_start, k_obs_end):
        # require lateral position y >= target_y during obstacle pass
        constraints += [x[0, k] >= target_y]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, verbose=False)

    print("Solve status:", prob.status)
    if prob.status in ["optimal", "optimal_inaccurate"]:
        x_np = np.array(x.value)
        u_np = np.array(u.value).reshape(-1)
        t = np.arange(N+1)*dt
        #plotting
        if plot_result.startswith('y'):
            fig, axs = plt.subplots(2,1, figsize=(9,6))
            axs[0].plot(t, x_np[0,:], marker='o', label='y (lateral position)')
            axs[0].axhline(target_y, color='k', linestyle='--', label='target lane center')
            axs[0].set_ylabel('lateral position y (m)')
            axs[0].legend()
            axs[1].step(np.arange(u_np.size)*dt, u_np, where='post', marker='o')
            axs[1].set_xlabel('time (s)')
            axs[1].set_ylabel('lateral acceleration u (m/s^2)')
            plt.tight_layout()
            plt.show()
        else:
            print("Plot suppressed by user.")
    else:
        print("Solve failed or infeasible.")

if __name__ == "__main__":
    main()