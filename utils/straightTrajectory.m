function [T_traj, p_dot_traj, p_ddot_traj] = straightTrajectory(T_start, T_end, s, s_dot, s_ddot)

% Info 
[p_start, R_start] = get_pR(T_start);
[p_end, R_end] = get_pR(T_end);
R_diff = logm(R_start' * R_end);
p_diff = p_end-p_start;
N = length(s);

% Loop to generate trajectory
T_traj = zeros(4,4,N);
p_dot_traj = zeros(3,N);
p_ddot_traj = zeros(3,N);

for i=1:N
    % Position
    p = p_start + s(i)*p_diff;
    R = R_start * expm(R_diff*s(i));
    T_traj(:,:,i) = [R,p;0 0 0 1];

    % Velocity 
    p_dot_traj(:,i) = s_dot(i)*p_diff;
    % w = R_start * A * expm(A*s(i)) * s_dot(i);

    % Acceleration
    p_ddot_traj(:,i) = s_ddot(i)*p_diff;
end