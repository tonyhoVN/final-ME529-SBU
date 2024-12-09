function T_sb = FK_BodyForm(B,M,theta)
% Calculate FK of open-chain robot by PoE w.r.t ee frame
    T_sb = M;
    num_j = length(theta); % Number of joints
    for i = 1:num_j
        % get exponential matrix 
        exp_B = vec2TF(B(:,i), theta(i));
        T_sb = T_sb*exp_B; 
    end
end