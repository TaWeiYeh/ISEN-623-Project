clear; clc; close all;
% give up
% required the coefficient of the objective function which has too many coefficients

options.screen = 'on';
options.printfile = 'toymin.out';
options.specsfile = which('sntoy.spc');

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
x0     = ones(4,1);
A      = [ 0 -4 -2 0];
b      = [ 0 ];
Aeq    = [];
beq    = [];

lb     = [  0,-Inf,-Inf,   0]';
ub     = Inf*ones(4,1);

options.name = 'toyprob';
options.stop = @toySTOP;

[x,fval,INFO,output,lambda,states] = snsolve(@toyObj, x0, A, b, Aeq, beq, lb, ub, ...
					     @toyCon, options);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [iAbort] = toySTOP(itn, nMajor, nMinor, condZHZ, obj, merit, step, ...
			      primalInf, dualInf, maxViol, maxViolRel, ...
			      x, xlow, xupp, xmul, xstate, ...
			      F, Flow, Fupp, Fmul, Fstate)

% Called every major iteration
% Use iAbort to stop SNOPT (if iAbort == 0, continue; else stop)

iAbort = 0