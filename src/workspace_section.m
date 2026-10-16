function [x, y, reachable] = workspace_section(p, height, extent, step)
%WORKSPACE_SECTION Horizontal section of the tool workspace in {S}.
%   [x, y, reachable] = WORKSPACE_SECTION(p, height, extent, step) checks a
%   grid of points at the given height above the ground, with x and y in
%   [-extent, extent] every step, and marks those reached by at least one
%   inverse kinematics solution within the joint limits. The lander deck
%   is neglected, as in the assignment.

[x, y] = meshgrid(-extent:step:extent);
P_S = [x(:).'; y(:).'; height*ones(1, numel(x))];
[~, ~, ok] = inverse_kinematics(station_to_base(P_S, p), p);
reachable = reshape(any(ok, 1), size(x));
end
