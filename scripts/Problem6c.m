%% 6. Trajectory
addpath("scripts\")
main;

% Start desire 
theta_start_d = [pi/2 pi/6 pi/3 pi/3 -pi/4 pi/2 pi/4]';
T_sd_start = FK_SpaceForm(S0, M0, theta_start_d);

% End desire 
theta_end_d = [pi/6 pi/4 0 pi/2 pi/2 pi/2 pi/3]';
T_sd_end = FK_SpaceForm(S0, M0, theta_end_d);

% Solve IK
theta0 = repmat(0.5,7,1);
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];
theta_start = theta_start_d;
theta_end = theta_end_d;
% Visualize 
figure(1);clf
show(robot,theta_start_d,'Visuals','on','Frames','on');
hold on
show(robot,theta_end_d,'Visuals','on','Frames','on');
hold off

%% 3rd order straight 

% Find v_max, a_max
s_vmax = 0.5;
T_vmax = straightTrajectory(T_sd_start,T_sd_end,s_vmax,0,0);
theta_vmax = IK_SpaceForm(T_vmax,theta_start,epsilon,S0,M0,jointLimit);
V_max = J_Geometry(S0,M0,theta_vmax)*jointVelLimit*ones(numJoint,1);
v_max = min(abs(V_max(4:end)));

s_amax = 0;
T_amax = straightTrajectory(T_sd_start,T_sd_end,s_amax,0,0);
theta_amax = IK_SpaceForm(T_amax,theta_start,epsilon,S0,M0,jointLimit);
A_max = J_Geometry(S0,M0,theta_amax)*jointAccLimit*ones(numJoint,1);
a_max = min(abs(A_max(4:end)));

% s Profile 
[s, s_dot, s_ddot, time_traj] = thirdOrderProfile(v_max,a_max);

% Straight trajectory in task space
[T_traj, p_dot_traj, p_ddot_traj] = straightTrajectory(T_sd_start,T_sd_end, s, s_dot, s_ddot);

% Joint space trajectory 
[joint_traj, joint_dot_traj, joint_ddot_traj] = inverseJointPath(T_traj, ...
    p_dot_traj, p_ddot_traj, theta_start, numJoint, S0, M0, epsilon, jointLimit);


% Visualize trajectory
visualizeTrajectory(robot, joint_traj, joint_dot_traj, joint_ddot_traj, ...
    time_traj, 2, S0, M0, true);
