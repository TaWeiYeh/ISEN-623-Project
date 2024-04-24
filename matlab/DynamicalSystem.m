classdef DynamicalSystem
	properties
		state_size
		control_size
		Q
		R
		Q_f
		goal
	end
	
	methods
		function obj = DynamicalSystem(state_size, control_size)
			obj.state_size = state_size;
			obj.control_size = control_size;
		end
		
		function obj = set_cost(obj, Q, R)
			obj.Q = Q;
			obj.R = R;
		end
		
		function obj = set_final_cost(obj, Q_f)
			obj.Q_f = Q_f;
		end
		
		function cost = calculate_cost(obj, x, u, varargin)
% 			if any(strcmp(fieldnames(varargin{:}), 'casadi'))
% 				cost = 0.5*(x-obj.goal)'*obj.Q*(x-obj.goal) + u'*obj.R*u;
% 			else
% 				cost = 0.5*(x-obj.goal)'*obj.Q*(x-obj.goal) + u'*obj.R*u;
%             end
            cost = 0.5*(x-obj.goal)'*obj.Q*(x-obj.goal) + u'*obj.R*u;
		end
		
		function final_cost = calculate_final_cost(obj, x, varargin)
% 			if any(strcmp(fieldnames(varargin{:}), 'casadi'))
% 				final_cost = 0.5*(x-obj.goal)'*obj.Q_f*(x-obj.goal);
% 			else
% 				final_cost = 0.5*(x-obj.goal)'*obj.Q_f*(x-obj.goal);
%             end
            final_cost = 0.5*(x-obj.goal)'*obj.Q_f*(x-obj.goal);
		end
		
		function obj = set_goal(obj, x_goal)
			obj.goal = x_goal;
		end
	end
end