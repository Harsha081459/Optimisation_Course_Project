#importing
import numpy as np
import cvxpy as cp

#default variables
dt = 0.1
N = 50
init_y = 0.0
init_vy = 0.0
target_y = 3.5
final_vy_target = 0.0
u_max = 3.0
v_max = 2.0
penalty_slack = 1000000.0

# Command line inputs for variables
# for dt :
while True:
    s = input("enter dt [Example format : 0.1] : (Press enter key for default value)").strip()
    if s == "":
        dt = 0.1
        break
    try:
        dt = float(s)
        if dt > 0:
            break
        print("dt must be > 0.")
    except:
        print("Invalid input. Try again.")

while True:
    s = input("enter N [Example format : 50 (integer >=1) ] : (Press enter key for default value)").strip()
    if s == "":
        N = 50
        break
    try:
        N = int(s)
        if N >= 1:
            break
        print("N must be integer >= 1.")
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter init_y [Example foramat : 0.0] : (Press enter key for default value)").strip()
    if s == "":
        init_y = 0.0
        break
    try:
        init_y = float(s); break
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter init_vy [Example format : 0.0] : (Press enter key for default value)").strip()
    if s == "":
        init_vy = 0.0
        break
    try:
        init_vy = float(s); break
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter target_y [Example format : 3.5] : (Press enter key for default value)").strip()
    if s == "":
        target_y = 3.5
        break
    try:
        target_y = float(s); break
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter final_vy_target [Example format : 0.0] : (Press enter key for default value)").strip()
    if s == "":
        final_vy_target = 0.0
        break
    try:
        final_vy_target = float(s); break
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter u_max [Example format : 3.0] : (Press enter key for default value)").strip()
    if s == "":
        u_max = 3.0
        break
    try:
        u_max = float(s)
        if u_max >= 0:
            break
        print("u_max must be >= 0.")
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter v_max [Example format : 2.0] (>=0) : (Press enter key for default value)").strip()
    if s == "":
        v_max = 2.0
        break
    try:
        v_max = float(s)
        if v_max >= 0:
            break
        print("v_max must be >= 0.")
    except:
        print("Invalid input. Try again.")

while True:
    s = input("Enter penalty_slack [Example format : 1000000] (>0) : (Press enter key for default value)").strip()
    if s == "":
        penalty_slack = 1e6
        break
    try:
        penalty_slack = float(s)
        if penalty_slack > 0:
            break
        print("penalty_slack must be > 0.")
    except:
        print("Invalid input. Try again.")

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