classdef CircleConstraintForDoubleIntegrator
	properties
		center
		r
		system
	end
	
	methods
		function obj = CircleConstraintForDoubleIntegrator(center, r, system)
			obj.center = center;
			obj.r = r;
			obj.system = system;
		end
		
		function constraint = evaluate_constraint(obj, x)
			% evolve the system for one to evaluate constraint
			x_next = obj.system.transition(x, zeros(size(obj.system.control_size)));
			length = (x_next(1) - obj.center(1))^2 + (x_next(2) - obj.center(2))^2;
			constraint = obj.r^2 - length;
		end
		
		function J = evaluate_constraint_J(obj, x)
			% evolve the system for one to evaluate constraint
			x_next = obj.system.transition(x, zeros(size(obj.system.control_size)));
			result = zeros(size(x));
			result(1) = -2*(x_next(1) - obj.center(1));
			result(2) = -2*(x_next(2) - obj.center(2));
			result(3) = -2*(x_next(1) - obj.center(1)) * obj.system.dt;
			result(4) = -2*(x_next(2) - obj.center(2)) * obj.system.dt;
			J = result;
		end
	end
end