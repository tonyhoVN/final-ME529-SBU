function T_mid = straightWayPoint(T_start, T_end, s) 
% Middle pose between T_start, T_end 

[p_start, R_start] = get_pR(T_start);
[p_end, R_end] =  get_pR(T_end);
p_mid = p_start + (p_end-p_start)*s;

R_mid = R_start*(expm(logm(R_start'*R_end)*s));
T_mid = [R_mid, p_mid; 0 0 0 1];