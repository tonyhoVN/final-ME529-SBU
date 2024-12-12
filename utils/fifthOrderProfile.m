function [s, s_dot, s_ddot, t] = fifthOrderProfile(vel_max, acc_max)
% 5th order Polynomial Position Profile. Choosing T so that 
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

    % Calculate minimum T required by velocity constraint
    T_vel = 1.875 / vel_max;   % from s_dot_max analysis
    % Calculate minimum T required by acceleration constraint
    T_acc = sqrt(5.8 / acc_max); % from s_ddot_max analysis
    
    % Choose T to satisfy both constraints
    T = max(T_vel, T_acc);
    
    % Time vector
    N = 100;
    t = linspace(0, T, N)';
    
    % Define normalized time variable x = t/T
    x = t / T;
    
    % Quintic polynomial
    s = 10*x.^3 - 15*x.^4 + 6*x.^5;
    
    % Velocity
    s_dot = (30*x.^2 - 60*x.^3 + 30*x.^4) / T;
    
    % Acceleration
    s_ddot = (60*x -180*x.^2 +120*x.^3) / (T^2);
end

