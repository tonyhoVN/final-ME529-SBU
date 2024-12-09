function S = screw2axis(s,q,h)
% Calculate screw axis representation S = [Sw; Sv]
% out: col vec
    % Normalize rotation vector 
    [s_hat, theta] = vec2unit(s);

    % Compute S_w, S_v
    if isinf(h)
        % Pure translation 
        S_w = zeros(3,1);
        S_v = s_hat;
    else
        % General case
        S_w = s_hat;
        S_v = cross(-s_hat, q) + h * s_hat;
    end

    % Combine to form the screw axis S
    S = [S_w; S_v];
end