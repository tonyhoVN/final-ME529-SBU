function theta = IK_SpaceForm(Tsd, theta0, epsilon, S0, M0)
% IK by  Newton Raphson method by body form
%   Tsd: desired TF w.r.t {s} frame
%   theta0: initial guess
%   epsilon: accuracy threshold
%   B0: screw axes matrix in home config
%   M0: {b} frame w.r.t {s} frame in home config 

max_iter = 1000; % Max iterations
lambda = 0.01; % learning rate

% Check input condition
if length(theta0) ~= length(S0) || length(epsilon) ~= 6
    error('Wrong size')
end

% Current config
theta = theta0; 

% HTM from current {s} frame to desire pose  
Tsb = FK_SpaceForm(S0, M0, theta); % FK at current config theta
Tbd = Tsb\Tsd; 

% Error screw in {b} frame (err = [err_bw; err_bv])
err_s = adjointMap(Tsb)*logTF(Tbd);

% Main loop 
k = 1;
while (k <= max_iter) && any(abs(err_s) >= epsilon)
    % Update theta
    theta = theta + lambda*pinv(J_SpaceForm(S0, theta))*err_s;

    % Update error
    Tsb = FK_SpaceForm(S0, M0, theta);
    Tbd = Tsb\Tsd; 
    err_s = adjointMap(Tsb)*logTF(Tbd);

    % Update k 
    k = k + 1;
end