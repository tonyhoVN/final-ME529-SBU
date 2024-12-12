function [joint_traj, joint_dot_traj, joint_ddot_traj] = inverseJointPath(T_traj, ...
    p_dot_traj, p_ddot_traj, theta_start, numJoint, S0, M0, epsilon, jointLimit)


[m,N] = size(p_dot_traj);
joint_traj = zeros(numJoint,N);
joint_dot_traj = zeros(numJoint,N);
joint_ddot_traj = zeros(numJoint,N);
theta = theta_start;

for i = 1:N
    % Inverse Kinematic
    theta = IK_SpaceForm(T_traj(:,:,i),theta,epsilon,S0,M0,jointLimit);
    joint_traj(:,i) = theta;
    % Inverse Velocity
    Jg = J_Geometry(S0,M0,theta);
    Jg = Jg(4:end,:); % only linear velocity 
    joint_dot_traj(:,i) = pinv(Jg)*p_dot_traj(:,i);
    % Inverse Acc
    joint_ddot_traj(:,i) = pinv(Jg)*p_ddot_traj(:,i);
end