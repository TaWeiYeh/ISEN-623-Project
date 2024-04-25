from scipy.optimize import minimize, Bounds
from systems import DoubleIntegrator, Car
import numpy as np
from constraints import CircleConstraintForDoubleIntegrator, CircleConstraintForCar
import time



def multiple_shooting_objective(u, x0, system, N):
    # calculate cost-to-go
    cost_to_go = 0
    x = x0
    for i in range(N - 1):
        # calculate cost
        u_k = u[i*system.control_size:(i+1)*system.control_size]
        cost_to_go = cost_to_go + system.calculate_cost(x, u_k)

        # calculate next state
        x = system.transition(x, u_k)

    # calculate final cost
    total_cost = cost_to_go + system.calculate_final_cost(x)
    return total_cost

def multiple_shooting_constraint(u, x0, system, N, contraint):
    x = x0
    constraint_values = np.zeros(N)
    for i in range(N - 1):
        # calculate constraint
        constraint_values[i] = constraint.evaluate_constraint(x)

        # calculate next state
        u_k = u[i*system.control_size:(i+1)*system.control_size]
        x = system.transition(x, u_k)
    
    constraint_values[N-1] = constraint.evaluate_constraint(x)
    constraint_values = -constraint_values # scipy optimizer requires inequality constraints to be non-negative
    return constraint_values

def constraint_goal(u, x0, system, N, goal):
    x = x0
    for i in range(N):
        # calculate next state
        u_k = u[i*system.control_size:(i+1)*system.control_size]
        x = system.transition(x, u_k)
    
    return x - goal

def predict_trajectory(x0, u, system):
    N = int(len(u) / system.control_size)
    x = np.zeros((system.state_size, N+1))
    x[:, 0] = x0
    for i in range(N):
        x[:, i+1] = system.transition(x[:, i], u[i*system.control_size:(i+1)*system.control_size])
    return x



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

# set the time horizon
T = 5               # time horizon
h = system.dt       # time step
N = int(T/h)        # number of time steps

# set the initial state
initial_state = np.array([0, 0, 0, 0])

# set the initial conntrol
initial_control = np.zeros(system.control_size * N)

# set goal
system.set_goal(np.array([3, 3, np.pi/2, 0]))

# set the bounds for the states
lb = np.array([-system.control_bound[0], -system.control_bound[1]] * N)
ub = np.array([system.control_bound[0], system.control_bound[1]] * N)
bnds = Bounds(lb, ub)

# set the cost function
system.set_cost(np.zeros((4, 4)), 0.05*np.identity(2))

# set the final cost function
Q_f = np.identity(4)
Q_f[0, 0] = 50
Q_f[1, 1] = 50
Q_f[2, 2] = 50
Q_f[3, 3] = 10
system.set_final_cost(Q_f)

# use multiple shooting method to create objective function
obj_fun = lambda x: multiple_shooting_objective(x, initial_state, system, N)

# set the constraints
constraint = CircleConstraintForCar(np.ones(2), 0.5, system)
constraint2 = CircleConstraintForCar(np.array([2, 2]), 1.0, system)
cons = ({'type': 'ineq', 'fun': lambda u: multiple_shooting_constraint(u, initial_state, system, N, constraint)}, 
        {'type': 'ineq', 'fun': lambda u: multiple_shooting_constraint(u, initial_state, system, N, constraint2)})
# cons = ({'type': 'ineq', 'fun': lambda u: multiple_shooting_constraint(u, initial_state, system, N, constraint2)})

# start timer
start = time.monotonic()

# create a solver
# res = minimize(obj_fun, initial_control, method='SLSQP', bounds=bnds)
res = minimize(obj_fun, initial_control, method='SLSQP', bounds=bnds, constraints=cons)

# end timer
end = time.monotonic()
print("Time elapsed during the process:", end - start)

# predict the trajectory
x = predict_trajectory(initial_state, res.x, system)

# draw the trajectory
# system.draw_trajectories(x)
system.draw_trajectories(x, constraints=[constraint, constraint2])
