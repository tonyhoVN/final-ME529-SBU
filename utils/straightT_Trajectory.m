function [T_traj, p_dot_traj, p_ddot_traj] = straightT_Trajectory(T_start, T_end, s, s_dot, s_ddot)

% Info 
T_diff = logm(T_start\T_end);

% path 
N = length(s);

% Loop to generate trajectory
T_traj = zeros(4,4,N);
p_dot_traj = zeros(3,N);
p_ddot_traj = zeros(3,N);

for i=1:N
    % position
    T = T_start*expm(T_diff*s(i));
    T_traj(:,:,i) = T;

    % vel
    T_dot = T*T_diff*s_dot(i);
    p_dot_traj(:,i) = T_dot(1:3,4);

    % acc
    T_ddot = T*T_diff^2*s_dot(i)^2 + T*T_diff*s_ddot(i);
    p_ddot_traj(:,i) = T_ddot(1:3,4);
end


