function ref = lspb_trajectory(Q, td, tb)
%LSPB_TRAJECTORY Linear segments with parabolic blends through path points.
%   ref = LSPB_TRAJECTORY(Q, td, tb) joins the path points Q (one column per
%   point, start and end included) with straight lines in joint space,
%   blended by parabolas of durations tb (one per path point); td are the
%   times between consecutive path points. ref(t) returns position,
%   velocity and acceleration (n x numel(t)) at any time t in [0, sum(td)].
%
%   As in Craig, Introduction to Robotics, chapter 7: the arm starts and
%   stops at rest, the blends at the ends lie inside the first and last
%   segment, and the via points are passed near, not through.
%   Each line passes through its two path points at the times
%   T = [tb(1)/2, interior path point times, tf - tb(end)/2], and the blend
%   at path point k, centred at T(k), changes the velocity from that of the
%   line before to that of the line after.

tb = tb(:).';
n_points = size(Q, 2);
tf = sum(td);
T = [0, cumsum(td)];
T(1) = tb(1)/2;
T(end) = tf - tb(end)/2;

% Line velocities, with rest before the first and after the last point
v = [zeros(size(Q, 1), 1), diff(Q, 1, 2) ./ diff(T), zeros(size(Q, 1), 1)];
a = diff(v, 1, 2) ./ tb;        % blend accelerations, one column per path point
if any(diff(T) < (tb(1:end-1) + tb(2:end))/2 - 1e-12)
    error('lspb_trajectory:blends', 'Blends overlap: shorten them.');
end

ref = @evaluate;

    function [q, qd, qdd] = evaluate(t)
        t = t(:).';
        n = size(Q, 1);
        % Default: on a line. Line k joins point k and k+1.
        k = discretize(t, [-inf, T(2:end-1), inf]);
        q   = Q(:, k) + v(:, k+1) .* (t - T(k));
        qd  = v(:, k+1) .* ones(n, 1);
        qdd = zeros(n, numel(t));
        % Blends override the lines around each path point
        for j = 1:n_points
            in = abs(t - T(j)) <= tb(j)/2 + 1e-9;           % tolerance: 10 - 9.85 > 0.15
            s = reshape(t(in), 1, []) - (T(j) - tb(j)/2);   % time since the blend started
            s = min(max(s, 0), tb(j));
            q_start = Q(:, j) - v(:, j) * tb(j)/2; % on the incoming line
            q(:, in)   = q_start + v(:, j) .* s + a(:, j)/2 .* s.^2;
            qd(:, in)  = v(:, j) + a(:, j) .* s;
            qdd(:, in) = a(:, j) .* ones(1, nnz(in));
        end
    end
end
