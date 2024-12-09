function Jb = J_BodyForm(B0, theta)
% Body Jacobian of n-DOF open chain robot  
% B and theta is column vector
    n = length(theta); % number of DOF
    Jb = zeros(size(B0),class(B0)); % initial value Jb
    Jb(:,n) = B0(:,n); % last column of Jb
    exp_matrix = eye(4); 
    for i = (n-1):-1:1
        exp_matrix = exp_matrix*vec2TF(B0(:,i+1), -theta(i+1));
        adj_matrix = adjointMap(exp_matrix);
        Jb(:,i) = adj_matrix*B0(:,i);
    end
end