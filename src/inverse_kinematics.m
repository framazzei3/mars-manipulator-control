function [q, elbow_up, within_limits] = inverse_kinematics(P_B, p)
%INVERSE_KINEMATICS Closed form inverse kinematics, all four solutions.
%   [q, elbow_up, within_limits] = INVERSE_KINEMATICS(P_B, p) for tool
%   positions P_B in {B} (3xN). q is 3x4xN, one solution per column:
%     1  azimuth towards the point,      sin(q3) > 0
%     2  azimuth towards the point,      sin(q3) < 0
%     3  azimuth away (arm over the top), sin(q3) > 0
%     4  azimuth away (arm over the top), sin(q3) < 0
%   elbow_up (4xN) marks the solutions with the elbow above the line from
%   the shoulder to the tool; within_limits (4xN) those that respect the
%   joint limits. Points out of reach give NaN and false.
%
%   Algebraic method: squaring and summing the three tool coordinates gives
%   cos(q3); the first two give q1; q2 then solves a 2x2 linear system in
%   cos(q2) and sin(q2), written here in closed form with atan2.

L1 = p.links.L(1);
L2 = p.links.L(2);
x = P_B(1,:); y = P_B(2,:); z = P_B(3,:);
N = size(P_B, 2);

c3 = (x.^2 + y.^2 + z.^2 - L1^2 - L2^2) / (2*L1*L2);
reachable = abs(c3) <= 1;
c3(~reachable) = NaN;

q = NaN(3, 4, N);
q1_towards = atan2(y, x);
q1_away    = atan2(-y, -x);
rho = hypot(x, y);
branches = {q1_towards,  rho, +1
            q1_towards,  rho, -1
            q1_away,    -rho, +1
            q1_away,    -rho, -1};
for k = 1:4
    [q1, r, sign_s3] = branches{k, :};       % r: tool distance along the arm plane
    s3 = sign_s3 * sqrt(1 - c3.^2);
    q3 = atan2(s3, c3);
    q2 = atan2(z, r) - atan2(L2*s3, L1 + L2*c3);
    q(:, k, :) = reshape([q1; wrap_to_pi(q2); q3], 3, 1, N);
end

% Elbow above the shoulder to tool line: -sin(q3) and the tool distance
% along the arm plane have the same sign
r_tool = [rho; rho; -rho; -rho];
s3_all = squeeze(sin(q(3, :, :)));
if N == 1
    s3_all = s3_all(:);
end
elbow_up = (-s3_all .* sign(r_tool)) > 0 & reachable;

within_limits = all(q >= p.q_min & q <= p.q_max, 1);
within_limits = reshape(within_limits, 4, N) & reachable;
end

function a = wrap_to_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
