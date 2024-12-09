function Js = J_SpaceForm(S0, theta)
% Space jacobian of n-DOF open chain robot 
% S0 and theta is column vector
    n = length(theta); % number of DOF
    Js = zeros(size(S0),class(S0)); % initial value of Js
    Js(:,1) = S0(:,1); % first column is axis vector
    exp_matrix = eye(4); 
    for i = 2:n 
        exp_matrix = exp_matrix*vec2TF(S0(:,i-1), theta(i-1));
        adj_matrix = adjointMap(exp_matrix);
        Js(:,i) = adj_matrix*S0(:,i);
    end
end