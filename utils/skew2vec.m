%% Skew to vector matrix
function v = skew2vec(w)
    % Ensure the input is a 3x3 skew-symmetric matrix
    diff = (w+w').^2;
    if size(w,1) ~= 3 || size(w,2) ~= 3 || sum(diff(:)) > 1e-6
        error('Input must be a 3x3 skew-symmetric matrix');
    end
    v = [w(3, 2); w(1, 3); w(2, 1)];
end