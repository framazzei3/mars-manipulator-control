function [q, within_limits] = elbow_up_solution(P_S, p)
%ELBOW_UP_SOLUTION Elbow up joint angles for a tool position in {S}.
%   [q, within_limits] = ELBOW_UP_SOLUTION(P_S, p) returns the elbow up
%   solution that respects the joint limits. If none does, it returns the
%   elbow up solution with the azimuth towards the point and
%   within_limits = false, so that the caller can decide what to do.
%   Elbow up keeps the arm clear of the lander deck when picking.

[Q, up, ok] = inverse_kinematics(station_to_base(P_S, p), p);
k = find(up & ok, 1);
within_limits = ~isempty(k);
if ~within_limits
    k = find(up(1:2), 1);
    if isempty(k)
        error('elbow_up_solution:unreachable', 'Position out of reach.');
    end
end
q = Q(:, k);
end
