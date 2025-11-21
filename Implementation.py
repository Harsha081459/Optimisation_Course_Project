# importing libraries
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

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
    dt = get_input("dt (s)", 0.1, float, lambda v: v > 0)
    N = get_input("N (steps, integer >=1)", 50, int, lambda v: v >= 1)
    init_y = get_input("init_y (m)", 0.0, float)
    init_vy = get_input("init_vy (m/s)", 0.0, float)
    target_y = get_input("target_y (m)", 3.5, float)
    final_vy_target = get_input("final_vy_target (m/s)", 0.0, float)
    u_max = get_input("u_max (m/s^2)", 3.0, float, lambda v: v >= 0)
    v_max = get_input("v_max (m/s)", 2.0, float, lambda v: v >= 0)
    jerk_max = get_input("jerk_max (m/s^3)", 2.0, float, lambda v: v >= 0)
    penalty_slack = get_input("penalty_slack", 1e6, float, lambda v: v > 0)
    plot_result = input("plot_result (y/n) [y]: ").strip().lower()
    if plot_result == "":
        plot_result = "y"
    obs_t_start = get_input("Obstacle start time (s)", 2.5, float, lambda v: v >= 0)
    obs_t_end   = get_input("Obstacle end time (s)", 4.5, float, lambda v: v > obs_t_start)

    # dynamic matrices
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([[0.5 * dt * dt], [dt]])

    # Decision variables
    u = cp.Variable((N, 1))
    x = cp.Variable((2, N+1))
    slack_u = cp.Variable((N, 1))
    slack_v = cp.Variable((N+1, 1))
    slack_final = cp.Variable((1, 1))

    constraints = [x[:, 0] == np.array([init_y, init_vy], dtype=float)]
    cost = 0

    # Store KKT constraints
    kkt_constraint = []

    for k in range(N):
        constraints += [x[:, k+1] == A @ x[:, k] + B[:, 0] * u[k, 0]]

        # Explicitly store u_max constraint
        c_u = (cp.abs(u[k, 0]) <= u_max + slack_u[k, 0])
        constraints += [c_u]
        kkt_constraint.append(c_u)

        if k > 0:
            constraints += [cp.abs(u[k, 0] - u[k-1, 0]) <= jerk_max]

        cost += cp.sum_squares(u[k, 0])

    for k in range(N+1):
        constraints += [cp.abs(x[1, k]) <= v_max + slack_v[k, 0]]

    constraints += [x[1, N] == final_vy_target]
    constraints += [x[0, N] == target_y + slack_final[0, 0]]
    constraints += [slack_u >= 0, slack_v >= 0, slack_final >= 0]

    cost += penalty_slack * (cp.sum_squares(slack_u) +
                             cp.sum_squares(slack_v) +
                             cp.sum_squares(slack_final))

    # Obstacle mapping
    k_obs_start = int(obs_t_start / dt)
    k_obs_end   = int(obs_t_end / dt)
    k_obs_start = max(0, min(N-1, k_obs_start))
    k_obs_end   = max(k_obs_start+1, min(N, k_obs_end))

    for k in range(k_obs_start, k_obs_end):
        constraints += [x[0, k] >= target_y]

    prob = cp.Problem(cp.Minimize(cost), constraints)

    # ---------------- OSQP timing ----------------
    t0 = time.time()
    prob.solve(solver=cp.OSQP, verbose=False)
    osqp_time = time.time() - t0
    print("\nOSQP Solve status:", prob.status)
    print(f"OSQP time taken = {osqp_time:.6f} seconds")

    x_osqp = np.array(x.value)
    u_osqp = np.array(u.value).reshape(-1)

    # ---------------- SCS timing ----------------
    t1 = time.time()
    prob.solve(solver=cp.SCS, verbose=False)
    scs_time = time.time() - t1
    print("\nSCS Solve status:", prob.status)
    print(f"SCS time taken  = {scs_time:.6f} seconds")

    x_scs = np.array(x.value)
    u_scs = np.array(u.value).reshape(-1)

    # -------------- Comparison summary --------------
    print("\n===== Solver Comparison =====")
    print(f"OSQP time: {osqp_time:.6f} s")
    print(f"SCS  time: {scs_time:.6f} s")
    if osqp_time < scs_time:
        print("OSQP is faster.")
    elif scs_time < osqp_time:
        print("SCS is faster.")
    else:
        print("Both equal.")
    print("================================")

    # -------------- Bar plot for solver times --------------
    solvers = ["OSQP", "SCS"]
    times = [osqp_time, scs_time]

    plt.figure(figsize=(6, 4))
    plt.bar(solvers, times)
    plt.ylabel("Time (seconds)")
    plt.title("Solver Runtime Comparison")
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.show()

    # -------------- Trajectory & control plot (both solvers) --------------
    if plot_result.startswith('y'):

        t = np.arange(N+1) * dt
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))

        # Trajectories
        axs[0].plot(t, x_osqp[0, :], marker='o', label='OSQP')
        axs[0].plot(t, x_scs[0, :], marker='x', label='SCS')
        axs[0].axhline(target_y, color='k', linestyle='--')

        rect = patches.Rectangle(
            (k_obs_start * dt, -10),
            (k_obs_end - k_obs_start) * dt,
            target_y + 10,
            color='red', alpha=0.3
        )
        axs[0].add_patch(rect)

        axs[0].set_ylabel("y (position)")
        axs[0].legend()
        axs[0].set_title("Trajectory Comparison: OSQP vs SCS")

        # Control inputs
        axs[1].step(t[:-1], u_osqp, where='post', label='OSQP', marker='o')
        axs[1].step(t[:-1], u_scs,  where='post', label='SCS', marker='x')
        axs[1].set_xlabel("time (s)")
        axs[1].set_ylabel("u (acc)")
        axs[1].legend()
        axs[1].set_title("Control Comparison")

        plt.tight_layout()
        plt.show()

    # -------------- KKT CHECK ----------------
    k_check = 5
    if k_check < len(kkt_constraint):
        c = kkt_constraint[k_check]
        dual = float(c.dual_value)
        primal_gap = float(c.violation())
        comp = primal_gap * dual

        print("\n===== KKT CHECK (k = 5) =====")
        print(f"Primal gap = {primal_gap}")
        print(f"Dual var   = {dual}")
        print(f"Slackness  = {comp}")
        print("================================")

if __name__ == "__main__":
    main()