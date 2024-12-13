%% 6. 5th oder Trajectory
addpath("scripts\")
main;

% Start desire 
theta_start_d = [pi/2 pi/6 pi/3 pi/3 -pi/4 pi/2 pi/4]';
T_sd_start = FK_SpaceForm(S0, M0, theta_start_d);

% End desire 
theta_end_d = [pi/6 pi/4 0 pi/2 pi/2 pi/2.5 pi/3]';
T_sd_end = FK_SpaceForm(S0, M0, theta_end_d);

% Solve IK
theta0 = repmat(0.5,7,1);
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];
theta_start = theta_start_d;
theta_end = theta_end_d;

% Visualize 
figure(1);clf
show(robot,theta_start_d,'Visuals','on','Frames','on');

%% 5th order circular path 
% Desire path parameters 
r = [0 1 1]';
r = r/norm(r);
d = T_sd_start(1:3,4) + [0.1 0.1 0.1]';

% Find v_max, a_max
s_vmax = 0.5;
T_vmax = circleTrajectory(T_sd_start,r,d,[s_vmax],[0.0],[0.0]);
theta_vmax = IK_SpaceForm(T_vmax,theta_start,epsilon,S0,M0,jointLimit);
V_max = J_Geometry(S0,M0,theta_vmax)*jointVelLimit*ones(numJoint,1);
v_max = min(abs(V_max(4:end)));
s_amax = 0.067;
T_amax = circleTrajectory(T_sd_start,r,d,[s_amax],[0.0],[0.0]);
theta_amax = IK_SpaceForm(T_amax,theta_start,epsilon,S0,M0,jointLimit);
A_max = J_Geometry(S0,M0,theta_amax)*jointAccLimit*ones(numJoint,1);
a_max = min(abs(A_max(4:end)));

% Time profile 
[s, s_dot, s_ddot, time_traj] = fifthOrderProfile(v_max,a_max);

% Circular profile 
[T_traj, p_dot_traj, p_ddot_traj] = circleTrajectory(T_sd_start, r, d, s, s_dot, s_ddot);

% IK to joint space 
[joint_traj, joint_dot_traj, joint_ddot_traj] = inverseJointPath(T_traj, ...
    p_dot_traj, p_ddot_traj, theta_start, numJoint, S0, M0, epsilon, jointLimit);

%%
visualizeTrajectory(robot, joint_traj, joint_dot_traj, joint_ddot_traj, ...
    time_traj, 2, S0, M0, false);

