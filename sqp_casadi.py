import casadi as ca
import numpy as np
from systems import DoubleIntegrator, Car
from constraints import CircleConstraintForDoubleIntegrator, CircleConstraintForCar



def multiple_shooting_objective(u, x0, system, N):
    # calculate cost-to-go
    cost_to_go = 0
    x = x0
    for i in range(N - 1):
        # calculate cost
        u_k = u[i*system.control_size:(i+1)*system.control_size]
        cost_to_go = cost_to_go + system.calculate_cost(x, u_k, casadi=True)

        # calculate next state
        x = system.transition(x, u_k, casadi=True)

    # calculate final cost
    total_cost = cost_to_go + system.calculate_final_cost(x, casadi=True)
    return total_cost

# Define the problem variables
# create a car system
# Ther are four states: 
# 1. x position
# 2. y position
# 3. heading angle
# 4. forward velocity
# There are two control inputs:
# 1. angular velocity (heading angle)
# 2. forward velocity (acceleration)
system = Car()

# Define the number of control intervals
T = 5               # Time horizon
dt = system.dt      # time step
N = int(T/dt) + 1   # number of time steps

# set goal
system.set_goal(np.array([3, 3, np.pi/2, 0]))

# set the cost function
system.set_cost(np.zeros((4, 4)), 0.05*np.identity(2))

# set the final cost function
Q_f = np.identity(4)
Q_f[0, 0] = 50
Q_f[1, 1] = 50
Q_f[2, 2] = 50
Q_f[3, 3] = 10
system.set_final_cost(Q_f)

# Define the initial state
initial_state = np.array([0, 0, 0, 0])

# Define the final state
final_state = np.array([3, 3, np.pi/2, 0])

# Create an optimization problem
opti = ca.Opti()

# Define the decision variables
X = opti.variable(system.state_size, N+1)  # State trajectory
U = opti.variable(system.control_size, N)  # Control trajectory

# Define the dynamics constraints
dt = system.dt       # time step
for k in range(N):
    k1 = system.dynamics(X[:, k], U[:, k], casadi=True)
    k2 = system.dynamics(X[:, k] + 0.5 * k1, U[:, k], casadi=True)
    k3 = system.dynamics(X[:, k] + 0.5 * k2, U[:, k], casadi=True)
    k4 = system.dynamics(X[:, k] + k3, U[:, k], casadi=True)
    x_next = X[:, k] + dt * (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    opti.subject_to(X[:, k+1] == x_next)

# Define the cost function
# cost = ca.sum1(ca.mtimes(U, U))  # Quadratic cost on controls
opti.minimize(multiple_shooting_objective(U, initial_state, system, N))
# opti.minimize(T); # race in minimal time

# Set the initial and final conditions
opti.subject_to(X[:, 0] == initial_state)  # Initial state
opti.subject_to(X[:, N] == final_state)  # Final state

# Set the control bounds
opti.subject_to(opti.bounded(-system.control_bound[0], U[0, :], system.control_bound[0]))
opti.subject_to(opti.bounded(-system.control_bound[1], U[1, :], system.control_bound[1]))

# set the constraints
constraint = CircleConstraintForCar(np.ones(2), 0.5, system)
constraint2 = CircleConstraintForCar(np.array([2, 2]), 1.0, system)
for i in range(N):
    opti.subject_to(constraint.evaluate_constraint(X[:, i], casadi=True) <= 0)
    opti.subject_to(constraint2.evaluate_constraint(X[:, i], casadi=True) <= 0)

# Set the solver options
## ipopt solver
# opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.tol": 1e-3}
# opti.solver("ipopt", opts)
## sqpmethod solver 
# opts = {"print_time": 0, "print_level": 0}
# opti.solver("sqpmethod", opts)        # failed to converge
## snopt solver
opts = {"print_time": 0, "print_level": 0}
opti.solver("osqp", opts)

# Solve the optimization problem
sol = opti.solve()

# Get the optimal solution
x_opt = sol.value(X)
u_opt = sol.value(U)

# # Print the optimal solution
# print("Optimal state trajectory:")
# print(x_opt)
# print("Optimal control trajectory:")
# print(u_opt)

# Plot the optimal solution
system.draw_trajectories(x_opt, constraints=[constraint, constraint2])