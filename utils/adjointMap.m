function Ad_T = adjointMap(T)
% Adjoint Representation of 𝑻
R = T(1:3,1:3);
p = T(1:3,4);
Ad_T = [R zeros(3,3); vec2skew(p)*R R];
end