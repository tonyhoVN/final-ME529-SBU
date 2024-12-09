%% Vector to skew matrix
function w = vec2skew(v)
    % Ensure the input is a 3x1 vector
    if length(v) ~= 3
        error('Input must be a 3-element vector');
    end
    w = [  0   -v(3)  v(2);
           v(3)   0   -v(1);
          -v(2)  v(1)   0];
end

