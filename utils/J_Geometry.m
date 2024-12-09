function J_g = J_Geometry(S0, M0, theta)
% Geometry Jacobian of open-chain link 
    n = length(theta); % number of joint
    
    % Space Jacobian
    Js = J_SpaceForm(S0, theta);
    
    % FK 
    Tsb = FK_SpaceForm(S0, M0, theta);
    p = Tsb(1:3,4);

    % Jg
    p_skew = skew(p);
    J_g = [eye(3), zeros(3,3); -p_skew, eye(3)]*Js;
end