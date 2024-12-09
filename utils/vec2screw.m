function [s,q,h] = vec2screw(S)
%Find screw (col) vector, position and pitch of screw axis
    
    S_w = S(1:3);
    S_v = S(4:6);
    
    % Check S_w
    if norm(S_w) < 1e-6
        % Pure translation
        s = S_v;
        h = inf;
        q = [0 0 0]';
    else
        % normal case
        h = S_w'*S_v;
        s = S_w;
        % find q
        syms q_x q_y q_z;
        q_ = [q_x q_y q_z]';
        eq1 = dot(q_,S_w) == 0; % q orthogonal with S_w
        eq2 = cross(q_,S_w) == S_v - h*S_w;
        sol = solve([eq1;eq2],[q_x;q_y;q_z]);
        q = [double(sol.q_x) double(sol.q_y) double(sol.q_z)]';
    end 
end