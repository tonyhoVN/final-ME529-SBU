function [w, theta] = vec2unit(v) 
% normalize w to unit vector

% theta in 
    % No rotation
    if norm(v) == 0 
        w = [0; 0; 1];
        theta = 0;
    else
        theta = norm(v);
        w = v/norm(v); 
    end
end
