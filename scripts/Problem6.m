%% 6. Trajectory
addpath("scripts\")
main;

% Start desire 
theta_start_d = [pi/2 pi/6 pi/2 pi/3 pi/5 pi/6 pi/4]';
T_sd_start = FK_SpaceForm(S0, M0, theta_start_d);

% End desire 
theta_end_d = [pi/6 pi/3 -pi/6 -pi/7 pi/4 pi/3 pi/2]';
T_sd_end = FK_SpaceForm(S0, M0, theta_end_d);

% Visualize 
figure(1);clf
show(robot,theta_start_d,'Visuals','on','Frames','on');
hold on
show(robot,theta_end_d,'Visuals','on','Frames','on');

%% Solve IK
theta0 = repmat(0.5,7,1);
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];
theta_start = IK_SpaceForm(T_sd_start,theta0,epsilon,S0,M0,jointLimit);
theta_end = IK_SpaceForm(T_sd_end,theta0,epsilon,S0,M0,jointLimit);

% Visualize 
show(robot,theta_start,'Visuals','on','Frames','on','Collisions','on');
show(robot,theta_end,'Visuals','on','Frames','on','Collisions','on');
hold off

%% Trapezoidal Trajectory
joint_traj = trapezoidal(theta_start, theta_end, jointVelLimit, jointAccLimit);

% Visualize trajectory
figure(2); clf
for i=1:length(joint_traj) % N: Number of samples
    theta_d = joint_traj(i,:)';
    show(robot,theta_d,'PreservePlot',false,'Visuals','on','Frames','on');
    hold on
    show(robot,theta_start,'Visuals','off','Frames','on','Collisions','on');
    show(robot,theta_end,'Visuals','off','Frames','on','Collisions','on');
    drawnow
end

