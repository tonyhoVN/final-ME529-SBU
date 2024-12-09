%% 4. Jacobian 
main;

% Angle 
% syms theta [7 1]
theta = [0 0 0 0 0 0 0]';

% Body Jacobian 
Jb = J_BodyForm(B0, theta);

% Geometry Jacobian 
Jg = J_Geometry(S0, M0, theta);