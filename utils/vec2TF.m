function T = vec2TF(S, theta)
%Generate tranformation from screw axis and rotation angle

% Detailed explanation goes here
S_w = S(1:3);
S_v = S(4:end);

% Find T
if norm(S_w) < 1e-6
    % Pure translation
    T = [eye(3), theta*S_v; 0 0 0 1];
else 
    % General case
    R = vec2rotMat(S_w, theta);
    skew_S_w = vec2skew(S_w);
    G= eye(3)*theta + (1-cos(theta))*skew_S_w + (theta-sin(theta))*skew_S_w*skew_S_w;
    p = G*S_v;
    T = [R, p; 0 0 0 1];
end
end
