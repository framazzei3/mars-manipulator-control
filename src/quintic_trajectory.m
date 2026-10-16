function ref = quintic_trajectory(Q, td)
%QUINTIC_TRAJECTORY Fifth order polynomials through path points, stopping at each.
%   ref = QUINTIC_TRAJECTORY(Q, td) joins the path points Q (one column per
%   point, start and end included) with fifth order polynomials of
%   durations td, with zero velocity and acceleration at every path point.
%   ref(t) returns position, velocity and acceleration (n x numel(t)) at
%   any time t in [0, sum(td)].

t_start = [0, cumsum(td(1:end-1))];
ref = @evaluate;

    function [q, qd, qdd] = evaluate(t)
        t = t(:).';
        n = size(Q, 1);
        q = zeros(n, numel(t)); qd = q; qdd = q;
        for k = 1:numel(td)
            if k < numel(td)
                in = t >= t_start(k) & t < t_start(k) + td(k);
            else
                in = t >= t_start(k);
            end
            T = td(k);
            s = min(reshape(t(in), 1, []) - t_start(k), T) / T;   % normalised time
            dq = Q(:, k+1) - Q(:, k);
            % Rest to rest quintic: q = q0 + dq (10 s^3 - 15 s^4 + 6 s^5)
            q(:, in)   = Q(:, k) + dq .* (10*s.^3 - 15*s.^4 + 6*s.^5);
            qd(:, in)  = dq .* (30*s.^2 - 60*s.^3 + 30*s.^4) / T;
            qdd(:, in) = dq .* (60*s - 180*s.^2 + 120*s.^3) / T^2;
        end
    end
end
