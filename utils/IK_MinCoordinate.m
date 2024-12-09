function theta = IK_MinCoordinate(X_d, theta0, epsilon, L)
% IK by  Newton Raphson method 
%   d: desired
%   theta0: initial guess
%   epsilon: accuracy 
%   L: length of link
    
    % Declare parameter
    max_iterations = 100; % iteration 
    r = 0.1; % learning rate 
    k = 0;
    theta = theta0; % solution
    e = X_d - FK_2R(theta,L); % error
    
    % main loop 
    while (k <= max_iterations) && (norm(e)>epsilon)
        % update theta
        theta = theta + r*pinv(J_2R(theta,L))*e;
        % update error 
        e = X_d - FK_2R(theta,L);
        % update iteration
        k = k+1;
    end

    % Clip solution in range -pi, pi
    theta = mod(theta + pi, 2*pi) - pi;
end