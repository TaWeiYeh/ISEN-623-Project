classdef DoubleIntegrator < DynamicalSystem
	properties
		dt
		control_bound
	end
	
	methods
		function obj = DoubleIntegrator()
			obj@DynamicalSystem(4, 2);
			obj.dt = 0.05;
			obj.control_bound = ones(2, 1) * 100;
			obj.goal = zeros(4, 1);
		end
		
		function result = transition(obj, x, u)
			result = zeros(4, 1);
			result(1:2) = x(1:2) + obj.dt * x(3:4);
			result(3:4) = x(3:4) + obj.dt * u;
		end
		
		function [A, B] = transition_J(obj, x, u)
			A = zeros(obj.state_size, obj.state_size);
			B = zeros(obj.state_size, obj.control_size);
			A(1:obj.state_size, 1:obj.state_size) = eye(obj.state_size);
			A(1, 3) = obj.dt;
			A(2, 4) = obj.dt;
			B(3, 1) = obj.dt;
			B(4, 2) = obj.dt;
		end
		
		function draw_trajectories(obj, x_trajectories)
			ax = subplot(111);
			circle1 = viscircles(ax, [1, 1], 0.5, 'Color', [0, 0.8, 0.8]);
			circle2 = viscircles(ax, [1.5, 2.2], 0.5, 'Color', [0, 0.8, 0.8]);
			scatter(ax, x_trajectories(1, 1:5:end), x_trajectories(2, 1:5:end), 4, 'r');
			axis(ax, 'equal');
			xlim(ax, [0, 3]);
			ylim(ax, [0, 3]);
			hold off;
		end
		
		function draw_u_trajectories(obj, u_trajectories)
			x = subplot(111);
			scatter(x, u_trajectories(1, 1:5:end), u_trajectories(2, 1:5:end), 4, 'r');
			hold off;
		end
	end
end