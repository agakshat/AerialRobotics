function [ u1, u2 ] = controller(~, state, des_state, params)
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

phic = 0;
kpz = 80;
kvz = 20;
kpy = 20;
kvy = 5;
kpo = 1000;
kvo = 10;
phic_dot = 0;
phic_ddot = 0;
u1 = params.mass*(params.gravity+des_state.acc(2)+kvz*(des_state.vel(2)-state.vel(2))+kpz*(des_state.pos(2)-state.pos(2)));
phic(end+1) = (-1.0/params.gravity)*(des_state.acc(1)+kvy*(des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
phic_dot(end+1) = phic(end)-phic(end-1);
phic_ddot(end+1) = phic_dot(end)-phic_dot(end-1);
u2 = params.Ixx*(phic_ddot(end)+kvo*(phic_dot(end)-state.omega(1))+kpo*(phic(end)-state.rot(1)));
%u2 = 0;
%u2 = params.Ixx*(phic_ddot+kvo*(phic_dot-state.omega(1))+kpo*(phic-state.rot(1)));
end

