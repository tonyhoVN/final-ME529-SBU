function E = logTF(T)
%Logarithm of transformation matrix 
    
    % Get R, p
    R = T(1:3,1:3);
    p = T(1:3,4);
    
    if abs(trace(R)-3) < 1e-6
        % No rotation
        S_w = zeros(3,1);
        [S_v,theta] = vec2unit(p);
    else
        % Normal case
        [S_w, theta] = rotMat2vec(R);
        S_w_skew = vec2skew(S_w);
        G_inv = (1/theta)*eye(3) - 1/2*S_w_skew + ...
                (1/theta - 1/2*cot(theta/2))*S_w_skew*S_w_skew;
        S_v = (G_inv*p);
    end


    % Comebine S_w and S_v        
    S = [S_w; S_v];
    
    % Multiply with theta
    E = S*theta;
end