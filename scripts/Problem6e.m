%% Problem 6e. Smooth trajectory 
addpath("scripts\")
main;

% Start desire 
theta_start_d = [pi/2 pi/6 pi/3 pi/3 -pi/4 pi/2 pi/4]';
T_sd_start = FK_SpaceForm(S0, M0, theta_start_d);

% Solve IK
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];
theta_start = theta_start_d;

% Waypoints
p1 = T_sd_start(1:3,4);
p2 = p1 + [0.1 0.1 0]';
p3 = p2 + [0.0 0.0 -0.2]';
p4 = p3 + [0.2 -0.2 -0.1]';
p = [p1,p2,p3,p4];
numPoint = 4;
startTime = 0;
endTime = 2;
N = 100;
timePoints = linspace(startTime,endTime,numPoint);
tSamples = linspace(startTime,endTime,N);

% Visualize 
figure(1);clf
show(robot,theta_start_d,'Visuals','on','Frames','on');
hold on
scatter3(p(1,:),p(2,:),p(3,:),'ro','filled')
hold off

%% 4 point Trajectory

% Loop
[n, numPoint] = size(p);
[p_start, R_start] = get_pR(T_sd_start);

T_traj = zeros(4,4,N);
p_traj = zeros(3,N);
p_dot_traj = zeros(3,N);
p_ddot_traj = zeros(3,N);

% Cubic Poly trajectory
[p_traj(1,:), p_dot_traj(1,:), p_ddot_traj(1,:)] = cubicpolytraj(p(1,:),timePoints,tSamples);
[p_traj(2,:), p_dot_traj(2,:), p_ddot_traj(2,:)] = cubicpolytraj(p(2,:),timePoints,tSamples);
[p_traj(3,:), p_dot_traj(3,:), p_ddot_traj(3,:)] = cubicpolytraj(p(3,:),timePoints,tSamples);

for i=1:N
    T_traj(:,:,i) = [R_start, p_traj(:,i); 0 0 0 1];
end

% time 
time_traj = tSamples;

% Joint space trajectory 
[joint_traj, joint_dot_traj, joint_ddot_traj] = inverseJointPath(T_traj, ...
    p_dot_traj, p_ddot_traj, theta_start, numJoint, S0, M0, epsilon, jointLimit);

% Visualize result
visualizeTrajectory(robot, joint_traj, joint_dot_traj, joint_ddot_traj, ...
    time_traj, 2, S0, M0, false);
