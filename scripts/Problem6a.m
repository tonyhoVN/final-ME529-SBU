%% 6. Trapezoidal Trajectory
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
hold on
show(robot,theta_end_d,'Visuals','on','Frames','on');
show(robot,theta_start,'Visuals','on','Frames','on','Collisions','on');
show(robot,theta_end,'Visuals','on','Frames','on','Collisions','on');
hold off

%% Trapezoidal Trajectory
[joint_traj, joint_dot_traj, joint_ddot_traj, time_trajectory] = trapezoidal(theta_start, theta_end, jointVelLimit, jointAccLimit);

% Visualize trajectory
visualizeTrajectory(robot, joint_traj', joint_dot_traj', joint_ddot_traj', ...
    time_trajectory, 2, S0, M0, false);


% % 
% [p, p_dot, p_ddot, time] = thirdOrderProfile(T_sd_start,T_sd_end,s_dot_max_p,s_ddot_max_p);
% figure()
% subplot(1,3,1)
% plot(time,p)
% subplot(1,3,2)
% plot(time,p_dot)
% subplot(1,3,3)
% plot(time,p_ddot)
