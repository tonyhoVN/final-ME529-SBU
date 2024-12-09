%% 2. Validate FK_SpaceForm 
addpath("scripts\")
main;

% Tsb by FK_SpaceForm
numJoint = 7; % number of joint
theta = [0 pi/3 pi/4 -pi/6 -pi/6 0 pi/2]';
T_sb = FK_SpaceForm(S0, M0, theta);
T_sb_1 = FK_BodyForm(B0, M0, theta);

% Visualize robot
figure(1); clf;
show(robot,theta,'Visuals','on','Frames','on');

% Draw Tsb
hold on 
triad('Matrix',T_sb,'Scale',0.2,'LineWidth',1,'linestyle','-');

% Validate by getTransform function 
T_sb_val = getTransform(robot, theta, eeFrame, baseFrame);