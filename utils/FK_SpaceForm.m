function T_sb = FK_SpaceForm(S,M,theta)
% Calculate FK of open-chain robot by PoE w.r.t {s} frame 
    T_sb = M;
    num_j = length(theta); % Number of joints
    for i = num_j:-1:1
        % get exponential matrix 
        exp_S = vec2TF(S(:,i), theta(i));
        T_sb = exp_S*T_sb; 
    end
end