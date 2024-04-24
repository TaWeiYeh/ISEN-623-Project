clear; clc; close all;

import casadi.*



% Define the problem variables
% create a car system
% There are four states: 
% 1. x position
% 2. y position
% 3. heading angle
% 4. forward velocity
% There are two control inputs:
% 1. angular velocity (heading angle)
% 2. forward velocity (acceleration)
system = Car;

% Define the number of control intervals
T = 5;              % Time horizon
dt = system.dt;     % time step
N = floor(T/dt) + 1;   % number of time steps

% set goal
system = system.set_goal([3; 3; pi/2; 0]);

% set the cost function
system = system.set_cost(zeros(4, 4), 0.05*eye(2));

% set the final cost function
Q_f = eye(4);
Q_f(1, 1) = 50;
Q_f(2, 2) = 50;
Q_f(3, 3) = 50;
Q_f(4, 4) = 10;
system = system.set_final_cost(Q_f);

% Define the initial state
initial_state = [0; 0; 0; 0];

% Define the final state
final_state = [3; 3; pi/2; 0];

% Create an optimization problem
opti = Opti();

% Define the decision variables
X = opti.variable(system.state_size, N+1);  % State trajectory
U = opti.variable(system.control_size, N);  % Control trajectory

% Define the dynamics constraints
dt = system.dt;       % time step
opts.casadi = true;
for k = 1:N
    k1 = system.dynamics(X(:, k), U(:, k), opts);
    k2 = system.dynamics(X(:, k) + 0.5 * k1, U(:, k), opts);
    k3 = system.dynamics(X(:, k) + 0.5 * k2, U(:, k), opts);
    k4 = system.dynamics(X(:, k) + k3, U(:, k), opts);
    x_next = X(:, k) + dt * (1/6) * (k1 + 2*k2 + 2*k3 + k4);
    opti.subject_to(X(:, k+1) == x_next);
end

% Define the cost function
opti.minimize(multiple_shooting_objective(U, opti.variable(4,1), system, N, opts));

% Set the initial and final conditions
opti.subject_to(X(:, 1) == initial_state);  % Initial state
opti.subject_to(X(:, N+1) == final_state);  % Final state

% Set the control bounds
opti.subject_to(-system.control_bound(1) <= U(1, :) <= system.control_bound(1));
opti.subject_to(-system.control_bound(2) <= U(2, :) <= system.control_bound(2));

% set the constraints
constraint = CircleConstraintForCar(ones(2), 0.5, system);
constraint2 = CircleConstraintForCar([2, 2], 1.0, system);
for i = 1:N
    opti.subject_to(constraint.evaluate_constraint(X(:, i), opts) <= 0);
    opti.subject_to(constraint2.evaluate_constraint(X(:, i), opts) <= 0);
end

% Set the solver options
% ----- ipopt solver -----
% ipoptopt = struct('bound_relax_factor', 0, 'max_iter', ...
%                 3000, 'max_cpu_time', 600, ...
%                 'print_level', 0);
% opt = struct('ipopt', ipoptopt);
% opti.solver('ipopt', opt);
% ----- sqp solver -----
opti.solver('snopt')


% Solve the optimization problem
sol = opti.solve();

% Get the optimal solution
x_opt = sol.value(X);
u_opt = sol.value(U);

% Print the optimal solution
% disp('Optimal state trajectory:');
% disp(x_opt);
% disp('Optimal control trajectory:');
% disp(u_opt);

% Plot the optimal solution
system.draw_trajectories(x_opt, [constraint, constraint2]);


function total_cost = multiple_shooting_objective(u, x0, system, N, varargin)
    % calculate cost-to-go
    cost_to_go = 0;
    x = x0;
    for i = 1:N-1
        % calculate cost
        u_k = u(:,i);
        cost_to_go = cost_to_go + system.calculate_cost(x, u_k, varargin{:});

        % calculate next state
        x = system.transition(x, u_k, varargin{:});
    end

    % calculate final cost
    total_cost = cost_to_go + system.calculate_final_cost(x, varargin{:});
end
