function [T_traj, p_dot_traj, p_ddot_traj] = circleTrajectory(T_start, r, d, s, s_dot, s_ddot)

[p_start, R_start] = get_pR(T_start);

% Compute circle center c
delta = p_start - d;
c = d + (dot(delta,r)*r);  % Project delta onto r

% Radius
rho = norm(p_start - c);

% Define local frame {o'} at c
x_prime = (p_start - c)/rho;  % unit vector from c to p_start
z_prime = r; % given circle axis as z'
y_prime = cross(z_prime, x_prime);

% Re-orthogonalize in case of numerical issues
x_prime = cross(y_prime, z_prime); 

% Normalize vector
x_prime = x_prime / norm(x_prime);
y_prime = y_prime/norm(y_prime);
z_prime = z_prime / norm(z_prime);

% Rotation matrix
R_so = [x_prime, y_prime, z_prime];

% Loop for trajectory 
T_traj = zeros(4,4,length(s));
p_dot_traj = zeros(3,length(s));
p_ddot_traj = zeros(3,length(s));

for i = 1:length(s)
    % position
    p_o = rho*[cos(2*pi*s(i)); sin(2*pi*s(i)); 0];
    p = c + R_so*p_o;
    T_traj(:,:,i) = [R_start, p; 0 0 0 1];
    
    % vel
    p_dot_o = rho*(2*pi)*s_dot(i)*[-sin(2*pi*s(i)); cos(2*pi*s(i)); 0];
    p_dot_traj(:,i) = R_so*p_dot_o;

    % acc
    p_ddot_o = rho*(2*pi)*s_ddot(i)*[-sin(2*pi*s(i)); cos(2*pi*s(i)); 0] + rho*(2*pi)^2*(s_dot(i)^2)*[-cos(2*pi*s(i)); -sin(2*pi*s(i)); 0];
    p_ddot_traj(:,i) = R_so*p_ddot_o;
end
