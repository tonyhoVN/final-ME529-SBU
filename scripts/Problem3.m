%% 3. Workspace
addpath("scripts\")
main;

% Contraint limit to range [-pi, pi]
jointLimit = max(min(jointLimit, pi), -pi)

% Generate all combinations of joints
N = 8;
jS = zeros(N, numJoint); % sample joint
for i = 1:numJoint
    joint_sample = linspace(jointLimit(1,i),jointLimit(2,i),N)';
    jS(:,i) = joint_sample; 
end

% Create ndgrid 
[j1,j2,j4,j6] = ndgrid(jS(:,1), jS(:,2), jS(:,4), jS(:,6));

% All possible combination of joint 2 4 6 
j1246 = [j1(:), j2(:), j4(:), j6(:)];

%% Plot ws x-z plane 
% Initial x,y,z
x_ws = [];
y_ws = [];
z_ws = [];

% FK to find ee possible in 3D space
for i = 1:length(j1246)
    theta = [j1246(i,1) j1246(i,2) 0 j1246(i,3) 0 j1246(i,4) 0];
    Tsb = FK_SpaceForm(S0,M0,theta);
    x_ws = [x_ws, Tsb(1,4)];
    y_ws = [y_ws, Tsb(2,4)];
    z_ws = [z_ws, Tsb(3,4)];
end

% x-z plane
figure(1);clf

% x-z plane
subplot(1,3,1);
scatter(x_ws,z_ws,'b');
xlabel('X range')
ylabel('Z range')
title('x-z Plane Workspace')
axis equal;
xlim([-1.5 1.5]); % Set x-axis limits
ylim([-1. 1.5]); % Set z-axis limits

% y-z plane
subplot(1,3,2);
scatter(y_ws,z_ws,'r');
xlabel('Y range')
ylabel('Z range')
title('y-z Plane Workspace')
axis equal;
xlim([-1.5 1.5]); % Set x-axis limits
ylim([-1. 1.5]); % Set z-axis limits

% x-y plane
subplot(1,3,3);
scatter(x_ws,y_ws,'c');
xlabel('X range')
ylabel('Y range')
title('x-y Plane Workspace')
axis equal;
xlim([-1.5 1.5]); % Set x-axis limits
ylim([-1.5 1.5]); % Set z-axis limits