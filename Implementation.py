#importing

import numpy as np
import cvxpy as cp

#initialise variables
dt = 0.1
N = 50
init_y = 0.0
init_vy = 0.0
target_y = 3.5
final_vy_target = 0.0
u_max = 3.0
v_max = 2.0
penalty_slack = 1000000.0

#dynamic matrices
A = np.array([[1.0, dt],
              [0.0, 1.0]])
B = np.array([[0.5 * dt * dt],
              [dt]])

# defining the required variables
u = cp.Variable((N, 1))
x = cp.Variable((2, N+1))
slack_u = cp.Variable((N,1))
slack_v = cp.Variable((N+1,1))
slack_final = cp.Variable((1,1))

# initialise the initial state
x0 = np.array([init_y, init_vy])
constraints = [x[:,0] == x0]

# initial cost
cost = 0
for k in range(N):
    constraints += [x[:, k+1] == A @ x[:, k] + B[:,0]*u[k,0]]
    constraints += [cp.abs(u[k,0]) <= u_max + slack_u[k,0]]
    cost += cp.sum_squares(u[k,0])

for k in range(N+1):
    constraints += [cp.abs(x[1,k]) <= v_max + slack_v[k,0]]

# final constraints
constraints += [x[1, N] == final_vy_target]
constraints += [x[0, N] == target_y + slack_final[0,0]]
constraints += [slack_u >= 0, slack_v >= 0, slack_final >= 0]
cost += penalty_slack * (cp.sum_squares(slack_u) + cp.sum_squares(slack_v) + cp.sum_squares(slack_final))

# defining the optimisation problem
prob = cp.Problem(cp.Minimize(cost), constraints)
#solving
prob.solve(solver=cp.OSQP, verbose=False)

# result
print("Status:", prob.status)
if prob.status in ["optimal", "optimal_inaccurate"]:
    print("u:", np.array(u.value).reshape(-1))
    print("slack_final:", np.array(slack_final.value).reshape(-1))
    print("y:", np.array(x.value)[0,:])
else:
    print("Solve failed.")