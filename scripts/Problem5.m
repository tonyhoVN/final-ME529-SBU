%% 5. Invert Kinematic 
addpath("scripts\")
main;

% Given desired HTM
theta_d = [0 -pi/3 -pi/4 -pi/4 -pi/3 pi/3 pi/2]';
Tsd = FK_SpaceForm(S0, M0, theta_d);

% Find IK by Body and Space Form
theta0 = [0 -pi/3 -pi/4 -pi/3 -pi/4 -pi/3 0]';
epsilon = [repmat(1e-3, 3, 1); repmat(1e-3, 3, 1)];
theta_b = IK_BodyForm(Tsd, theta0, epsilon, B0, M0, jointLimit);
theta_s = IK_SpaceForm(Tsd, theta0, epsilon, S0, M0, jointLimit);

%% Visualize IK solutions
figure(1); clf
show(robot,theta_d,'Visuals','on','Frames','on','Collisions','off');
hold on 
show(robot,theta_b,'Visuals','on','Frames','on','Collisions','on');
xlim([-1 1])
ylim([-1 1])
zlim([0 1])

figure(2); clf
show(robot,theta_d,'Visuals','on','Frames','on','Collisions','off');
hold on 
show(robot,theta_s,'Visuals','on','Frames','on','Collisions','on');
xlim([-1 1])
ylim([-1 1])
zlim([0 1])

%% Validate by FK 
T_sb = FK_SpaceForm(S0, M0, theta_s);
T_err = T_sb - Tsd

