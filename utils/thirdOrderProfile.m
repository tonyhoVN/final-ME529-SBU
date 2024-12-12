function [s, s_dot, s_ddot, t] = thirdOrderProfile(vel_max, acc_max)
% 3rd order Polynomial Position Profile. Choosing T so that 
% s_dot_max < vel_max and s_ddot_max < acc_max.
%
% INPUTS:
%   vel_max : maximum allowed velocity
%   acc_max : maximum allowed acceleration
%
% OUTPUTS:
%   t       : time vector from [0, T]
%   s       : position trajectory (size N)
%   s_dot   : velocity trajectory (size N)
%   s_ddot  : acceleration trajectory (size N)

% Find maximum T satifying both vel and acc in all axis
T_vel = (3/2)/vel_max;
T_acc = sqrt(6/acc_max);
T = max(T_vel, T_acc);

% Vector time 
N = 100;
t = linspace(0,T,N)';

% s profile
s = (3/T^2)*t.^2 - (2/T^3)*t.^3;
s_dot = (6/T^2)*t - (6/T^3)*t.^2;
s_ddot = 6/T^2 - (12/T^3)*t;
