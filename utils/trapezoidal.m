function [theta_trajectory, theta_dot_trajectory, theta_ddot_trajectory, t_trajectory] = ... 
    trapezoidal(start, final, jointVelLimit, jointAccLimit)
% Generate trapezoidal trajectory 
%   start: start point 
%   final: end point 
%   jointVelLimit

numJoint = length(start);
theta_trajectory = [];
theta_dot_trajectory = [];
theta_ddot_trajectory = [];
N = 100;

% Loop for each joint
for j = 1:numJoint
    % Calculate the distance
    theta_diff = final(j) - start(j);
    
    % Select maximum vel and acc, calculate t_acc
    v_max = abs(jointVelLimit/theta_diff);
    a_max = abs(jointAccLimit/theta_diff);
    
    % Check if v_max can be reached
    if v_max^2/a_max >= 1
        % Cannot reach v_max
        t_acc = sqrt(1/a_max);
        v_max = a_max*t_acc;
        t_vel = 0;
    else
        % reach v_max
        t_acc = v_max/a_max;
        t_vel = 1/abs(v_max) - t_acc;
    end

    % Total time
    T = t_acc*2 + t_vel;

    % Generate profile s,s_d,s_dd
    t_trajectory = linspace(0,T,N);
    s = zeros(length(t_trajectory),1);
    s_dot = zeros(length(t_trajectory),1);
    s_ddot = zeros(length(t_trajectory),1);
    for i = 1:length(t_trajectory)
        t = t_trajectory(i); % Current time 
        if t <= t_acc
            s(i) = a_max*t^2/2;
            s_dot(i) = a_max*t;
            s_ddot(i) = a_max;
        elseif (t_acc < t) && (t <= (T-t_acc))
            s(i) = v_max*t - v_max^2/(2*a_max);
            s_dot(i) = v_max;
            s_ddot(i) = 0;
        elseif ((T-t_acc) < t) && (t <= T)
            s(i) = (2*a_max*v_max*T - 2*v_max^2 - (a_max^2)*(t-T)^2) / (2*a_max);
            s_dot(i) = -a_max*(t-T);
            s_ddot(i) = -a_max;
        end
    end
    
    % Compute joint trajectory
    theta_j = start(j) + s*theta_diff;
    theta_trajectory = [theta_trajectory, theta_j];
    theta_j_dot = s_dot*theta_diff;
    theta_dot_trajectory = [theta_dot_trajectory, theta_j_dot];
    theta_j_ddot = s_ddot*theta_diff;
    theta_ddot_trajectory = [theta_ddot_trajectory, theta_j_ddot];
end