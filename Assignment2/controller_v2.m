function [ u1, u2 ] = controller_v2(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% u1 = 0;
% u2 = 0;

% FILL IN YOUR CODE HERE

% Proportional and Derivative Gains
Kp_z = 80;
Kd_z = 20;

Kp_y = 20;
Kd_y = 5;

Kp_a = 1000;
Kd_a = 10;

u1 = params.mass*(params.gravity + des_state.acc(2,1) + Kd_z*(des_state.vel(2,1)-state.vel(2,1)) + Kp_z*(des_state.pos(2,1)-state.pos(2,1)));

phi_c = -(1/params.gravity)*(des_state.acc(1,1) + Kd_y*(des_state.vel(1,1)-state.vel(1,1) + Kp_y*(des_state.pos(1,1)-state.pos(1,1))));
phi_c_dot = 0;
phi_c_2dot = 0;

u2 = params.Ixx*(phi_c_2dot + Kd_a*(phi_c_dot-state.omega) + Kp_a*(phi_c-state.rot));
%u2 = (Kd_a*(phi_c_dot-state.omega) + Kp_a*(phi_c-state.rot));
%u2 = (phi_c_2dot + Kd_a*(phi_c_dot-state.omega) + Kp_a*(phi_c-state.rot));

% Hover Controller
% u1 = params.mass*(params.gravity + Kd_z*(-state.vel(2,1)) + Kp_z*(des_state.pos(2,1)-state.pos(2,1)));
% 
% phi_c = -(1/params.gravity)*(Kd_y*(-state.vel(1,1) + Kp_y*(des_state.pos(1,1)-state.pos(1,1))));
% phi_c_dot   = 0;
% phi_c_2dot  = 0;
% 
% u2 = params.Ixx*(phi_c_2dot + Kd_a*(phi_c_dot-state.omega) + Kp_a*(phi_c-state.rot));
end
