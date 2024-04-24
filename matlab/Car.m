classdef Car < DynamicalSystem
	properties
		dt
		control_bound
	end
	
	methods
		function obj = Car()
			obj@DynamicalSystem(4, 2);
			obj.dt = 0.05;
			obj.control_bound = [pi/2; 10];
			obj.goal = zeros(4, 1);
		end
		
		function x_next = runge_kutta_integration(obj, f, x, u, varargin)
			k1 = f(x, u, varargin{:});
			k2 = f(x + 0.5 * obj.dt * k1, u, varargin{:});
			k3 = f(x + 0.5 * obj.dt * k2, u, varargin{:});
			k4 = f(x + obj.dt * k3, u, varargin{:});
			x_next = x + obj.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
		end
		
		function dxdt = dynamics(obj, x, u, varargin)
            import casadi.*
			if any(strcmp(fieldnames(varargin{:}), 'casadi'))
				dxdt = x;
				dxdt(1) = x(4) * sin(x(3));
				dxdt(2) = x(4) * cos(x(3));
				dxdt(3) = u(2) * x(4);
				dxdt(4) = u(1);
			else
				dxdt = zeros(4, 1);
				dxdt(1) = x(4) * sin(x(3));
				dxdt(2) = x(4) * cos(x(3));
				dxdt(3) = u(2) * x(4);
				dxdt(4) = u(1);
			end
		end
		
		function x_next = transition(obj, x, u, varargin)
			x_next = obj.runge_kutta_integration(@obj.dynamics, x, u, varargin{:});
		end
		
		function [A, B] = transition_J(obj, x, u)
			A = eye(obj.state_size);
			B = zeros(obj.state_size, obj.control_size);
			A(1, 4) = sin(x(3)) * obj.dt;
			A(1, 3) = x(4) * cos(x(3)) * obj.dt;
			A(2, 4) = cos(x(3)) * obj.dt;
			A(2, 3) = -x(4) * sin(x(3)) * obj.dt;
			A(3, 4) = u(2) * obj.dt;
			B(3, 2) = x(4) * obj.dt;
			B(4, 1) = obj.dt;
		end
		
		function draw_trajectories(obj, x_trajectories, varargin)
            figure("Name", "Result")
			ax = subplot(1,1,1);
            hold on;
			if any(strcmp(fieldnames(varargin{:}), 'casadi'))
				constraints = varargin{find(strcmp(varargin, 'constraints')) + 1};
				for i = 1:numel(constraints)
					circle = viscircles(ax, constraints{i}.center, constraints{i}.r, 'Color', [0, 0.8, 0.8]);
				end
			else
				circle1 = viscircles(ax, [1, 1], 0.5, 'Color', [0, 0.8, 0.8]);
				circle2 = viscircles(ax, [2, 2], 1, 'Color', [0, 0.8, 0.8]);
			end
            drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );
			for i = 1:5:size(x_trajectories, 2)-1
				circle_car = viscircles(ax, [x_trajectories(1, i), x_trajectories(2, i)], 0.1, 'Color', 'none');
                dx = [x_trajectories(1, i), x_trajectories(1, i) + 0.1*cos(x_trajectories(3, i))];
                dy = [x_trajectories(2, i), x_trajectories(2, i) + 0.1*sin(x_trajectories(3, i))];
				drawArrow(dx, dy,'linewidth',3,'color','r');
			end
			axis(ax, 'equal');
			xlim(ax, [-1, 4]);
			ylim(ax, [-1, 4]);
			hold off;
		end
	end
end