%% 4. Jacobian 
addpath("scripts\")
main;

% Angle 
% syms a [7 1]
% theta = [pi/2 pi/6 0 0 0 pi/2 0]',
theta = [0 pi/6 0 pi/3 0 0 0]';

% Visualize 
figure(1);clf
show(robot,theta,'Visuals','on','Frames','on');

% Body Jacobian 
Jb = J_BodyForm(B0, theta);
det(Jb'*Jb)


