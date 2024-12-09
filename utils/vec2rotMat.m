function R = vec2rotMat(w, theta)
    % Ensure w is a unit vector
    if abs(norm(w)-1) > 1e-6 
        warning('vector is not a unit vector. Convert to unit vector');
        [w_unit,~] = vec2unit(w);
        w = w_unit;
    end

    % Predefine parameters
    w_skew = vec2skew(w);
    cos_t = cos(theta);
    sin_t = sin(theta);

    % Compute the rotation matrix 
    R = eye(3) + sin_t*w_skew + (1-cos_t)*w_skew*w_skew;
end