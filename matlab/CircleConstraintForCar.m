classdef CircleConstraintForCar
	properties
		center
		r
		system
	end
	
	methods
		function obj = CircleConstraintForCar(center, r, system)
			obj.center = center;
			obj.r = r;
			obj.system = system;
		end
		
		function constraint = evaluate_constraint(obj, x, varargin)
			% evolve the system for one to evaluate constraint
			% x_next = obj.system.transition(x, zeros(obj.system.control_size), varargin{:});
            x_next = x;
			length = (x_next(1) - obj.center(1))^2 + (x_next(2) - obj.center(2))^2;
			constraint = obj.r^2 - length;
		end
		
		function J = evaluate_constraint_J(obj, x, varargin)
			% evolve the system for one to evaluate constraint
			% x_next = obj.system.transition(x, zeros(obj.system.control_size), varargin{:});
            x_next = x;
			result = zeros(size(x));
			result(1) = -2*(x_next(1) - obj.center(1));
			result(2) = -2*(x_next(2) - obj.center(2));
			result(3) = -2*(x_next(1) - obj.center(1)) * obj.system.dt * x(3) * cos(x(2)) + 2*(x_next(2) - obj.center(2)) * obj.system.dt * x(3) * sin(x(2));
			result(4) = -2*(x_next(2) - obj.center(2)) * obj.system.dt * cos(x(2)) - 2*(x_next(1) - obj.center(1)) * obj.system.dt * sin(x(2));
			J = result;
		end
	end
end