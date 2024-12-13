addpath("scripts\")
main;

% Solve IK
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];

% Start desire 
theta_start = [pi/2 pi/6 pi/3 pi/3 -pi/4 pi/2 pi/4]';
T_start = FK_SpaceForm(S0, M0, theta_start);
[p_start, R_start] = get_pR(T_start);
numPoint = 20;
startTime = 0;
endTime = 5;
N = 100;
timePoints = linspace(startTime,endTime,numPoint);
tSamples = linspace(startTime,endTime,N);

% waypoint
p = generate_S_waypoints(p_start(1),p_start(2),p_start(3),0.5,0.1,numPoint)';

% Loop
[n, numPoint] = size(p);
T_traj = zeros(4,4,N);
p_traj = zeros(3,N);
p_dot_traj = zeros(3,N);
p_ddot_traj = zeros(3,N);

% vel = 0.5*ones(3,numPoint);
% vel(3,:) = zeros(1,numPoint);
vel = zeros(3,2);

% Cubic Poly trajectory
% [p_traj(1,:), p_dot_traj(1,:), p_ddot_traj(1,:)] = bsplinepolytraj(p(1,:),timePoints,tSamples);
% [p_traj(2,:), p_dot_traj(2,:), p_ddot_traj(2,:)] = bsplinepolytraj(p(2,:),timePoints,tSamples);
% [p_traj(3,:), p_dot_traj(3,:), p_ddot_traj(3,:)] = bsplinepolytraj(p(3,:),timePoints,tSamples);
[p_traj, p_dot_traj, p_ddot_traj] = minjerkpolytraj(p, timePoints,100);

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
