function [w_unit, theta] = rotMat2vec(R)
%     trR = trace(R);
%     r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
%     r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
%     r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
%     
%     % Case 1
%     if abs(trR - 3) < 1e-6 || isequal(R, eye(3))
%         theta = 0;
%         w = [0 0 1]';
%         return;
%     end
%     
%     % Case 2
%     if abs(trR + 1) < 1e-6
%         theta = pi;
%         % Choose the axis based on which diagonal element is the largest
%         if r11 > r22 && r11 > r33
%             w = (1/sqrt(2*(1+r11)))*[1+r11; r21; r31];
%         elseif r22 > r11 && r22 > r33
%             w = (1/sqrt(2*(1+r22)))*[r12; 1+r22; r32];
%         else
%             w = (1/sqrt(2*(1+r33)))*[r13; r23; 1+r33];
%         end
%         return;
%     end
% 
%     % Case3
%     theta = acos((trR-1)/2);
%     w = (1/(2*sin(theta))) * [r32 - r23; r13 - r31; r21 - r12];
    
    % Find skew matrix by logarithm  
    skew_w = logm(R);
    % Convert to vector from skew matrix
    w = skew2vec(skew_w);
    % Compute unit vector and rotation angle
    [w_unit, theta] = vec2unit(w);
end