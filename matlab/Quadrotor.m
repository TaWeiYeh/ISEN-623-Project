classdef Quadrotor < DynamicalSystem
	properties
		dt
		control_bound
	end
	
	methods
		function obj = Quadrotor()
			obj@DynamicalSystem(12, 4);
			obj.dt = 0.02;
			obj.control_bound = [10; 10; 10; 10];
			obj.goal = zeros(12, 1);
		end
		
		function x_next = transition(obj, x, u)
			forces = [0; 0; u(1)+u(2)+u(3)+u(4)];
			torques = [u(1)-u(3); u(2)-u(4); u(1)-u(2)+u(3)-u(4)];
			rotation_matrix = obj.get_rotation_matrix(x);
			J_omega = obj.get_J_omega(x);
			g = [0; 0; -10];
			x_next = zeros(12, 1);
			x_next(1:3) = x(1:3) + obj.dt * x(7:9);
			x_next(7:9) = x(7:9) + obj.dt * (g + rotation_matrix*forces - 0 * x(7:9));
			x_next(4:6) = x(4:6) + obj.dt * J_omega*x(10:12);
			x_next(10:12) = x(10:12) + obj.dt * torques;
		end
		
		function [A, B] = transition_J(obj, x, u)
			A = zeros(obj.state_size, obj.state_size);
			B = zeros(obj.state_size, obj.control_size);
			u_sum = u(1)+u(2)+u(3)+u(4);
			rotation_matrix = obj.get_rotation_matrix(x);
			A(1:obj.state_size, 1:obj.state_size) = eye(obj.state_size);
			A(1, 7) = 1 * obj.dt;
			A(2, 8) = 1 * obj.dt;
			A(3, 9) = 1 * obj.dt;
			
			A(7, 4) = u_sum * obj.dt * (sin(x(4)) * cos(x(3)));
			A(7, 5) = u_sum * obj.dt * (cos(x(4)) * sin(x(3)));
			A(8, 5) = u_sum * obj.dt * (-cos(x(5)) * cos(x(4)));
			A(8, 6) = u_sum * obj.dt * (sin(x(5)) * sin(x(4)));
			A(9, 5) = u_sum * obj.dt * (-sin(x(4)));
			
			A(3, 3) = (x(10) * cos(x(3)) * tan(x(4)) - x(11) * sin(x(3)) * tan(x(4))) * obj.dt;
			A(3, 4) = 1.0 / (cos(x(4))^2) * (x(10) * sin(x(3)) + x(11) * cos(x(3))) * obj.dt;
			A(3, 10) = 1 * obj.dt;
			A(3, 11) = sin(x(3)) * tan(x(4)) * obj.dt;
			A(3, 12) = cos(x(3)) * tan(x(4)) * obj.dt;
			A(4, 3) = (-sin(x(3)) * x(10) - cos(x(3)) * x(11)) * obj.dt;
			A(4, 10) = cos(x(3)) * obj.dt;
			A(4, 11) = -sin(x(3)) * obj.dt;
			A(5, 3) = (cos(x(3)) / cos(x(4)) * x(10) - sin(x(3)) / cos(x(4)) * x(11)) * obj.dt;
			A(5, 4) = sin(x(4)) / (cos(x(4))^2) * (sin(x(3)) * x(10) + cos(x(3)) * x(11)) * obj.dt;
			A(5, 10) = sin(x(3)) / cos(x(4)) * obj.dt;
			A(5, 11) = cos(x(3)) / cos(x(4)) * obj.dt;
			
			torque_matrix = zeros(3, 4);
			torque_matrix(1, 1) = 1;
			torque_matrix(1, 3) = -1;
			torque_matrix(2, 2) = 1;
			torque_matrix(2, 4) = -1;
			torque_matrix(3, 1) = 1;
			torque_matrix(3, 2) = -1;
			torque_matrix(3, 3) = 1;
			torque_matrix(3, 4) = -1;
			
			force_matrix = zeros(3, 4);
			force_matrix(3, :) = 1;
			
			B(10:12, :) = torque_matrix * obj.dt;
			B(7:9, :) = obj.dt * rotation_matrix * force_matrix;
		end
		
		function R = get_rotation_matrix(obj, x)
			R = zeros(3, 3);
			R(1, 1) = cos(x(3)) * cos(x(5)) - cos(x(4)) * sin(x(3)) * sin(x(5));
			R(1, 2) = -cos(x(3)) * sin(x(3)) - cos(x(3)) * cos(x(4)) * sin(x(5));
			R(1, 3) = sin(x(4)) * sin(x(3));
			R(2, 1) = cos(x(4)) * cos(x(5)) * sin(x(3));
			R(2, 2) = cos(x(3)) * cos(x(4)) * cos(x(5)) - sin(x(3)) * sin(x(5));
			R(2, 3) = -cos(x(5)) * sin(x(4));
			R(3, 1) = sin(x(3)) * sin(x(4));
			R(3, 2) = cos(x(3)) * sin(x(4));
			R(3, 3) = cos(x(4));
		end
		
		function J_omega = get_J_omega(obj, x)
			J_omega = zeros(3, 3);
			J_omega(1, 1) = 1;
			J_omega(1, 2) = sin(x(3)) * tan(x(4));
			J_omega(1, 3) = cos(x(3)) * tan(x(4));
			J_omega(2, 2) = cos(x(3));
			J_omega(2, 3) = -sin(x(3));
			J_omega(3, 2) = sin(x(3)) / cos(x(4));
			J_omega(3, 3) = cos(x(3)) / cos(x(4));
		end
	end
end