function theta = IK_BodyForm(Tsd, theta0, epsilon, B0, M0, jointLimit)
% IK by  Newton Raphson method by body form
%   Tsd: desired TF w.r.t {s} frame
%   theta0: initial guess
%   epsilon: accuracy threshold
%   B0: screw axes matrix in home config
%   M0: {b} frame w.r.t {s} frame in home config 

max_iter = 1000; % Max iterations
lambda = 0.01; % learning rate

% Check input condition
if length(theta0) ~= length(B0) || length(epsilon) ~= 6
    error('Wrong size')
end

% Current config
theta = theta0; 

% HTM from current {b} frame to desire pose  
Tsb = FK_BodyForm(B0, M0, theta); % FK at current config theta
Tbd = Tsb\Tsd; 

% Error screw in {b} frame (err = [err_bw; err_bv])
err_b = logTF(Tbd);

% Main loop 
k = 1;
while (k <= max_iter) && any(abs(err_b) >= epsilon)
    % Update theta
    theta = theta + lambda*pinv(J_BodyForm(B0, theta))*err_b;

    % Update error
    Tsb = FK_BodyForm(B0, M0, theta); % FK at current config theta
    Tbd = Tsb\Tsd; 
    err_b = logTF(Tbd);

    % Update k 
    k = k + 1;
end

% Constraint solution in range [-pi, pi]
theta = mod(theta + pi, 2*pi) - pi;

% Check with jointLimit 
theta_origin = theta;
theta = min(max(theta, jointLimit(1,:)'), jointLimit(2,:)');
if ~isequal(theta_origin, theta)
    warning("solution exceed joint limit")
end

