function waypoints = generate_S_waypoints(x0, y0, z0, L, A, N)
% generate_S_waypoints creates a set of waypoints resembling an "S" curve.
%
% INPUTS:
%   x0, y0, z0 : initial coordinates of the starting point
%   L          : total length in Y-direction to draw the "S"
%   A          : amplitude of the "S" in the X-direction
%   N          : number of waypoints
%
% OUTPUT:
%   waypoints : an Nx3 matrix of [x, y, z] positions

    % Parameter t goes from 0 to 1
    t = linspace(0, 1, N)'; 

    % Define the parametric equations for the "S":
    % Full sine wave from 0 to 1: X(t)=x0 + A*sin(2*pi*t)
    x = x0 + A * sin(2*pi*t);
    % Move along Y from y0 to y0+L
    y = y0 + L * t;
    % Keep Z constant
    z = repmat(z0, N, 1);

    % Combine into waypoints array
    waypoints = [x, y, z];
end
