function [P_tool, P_elbow, T_BT] = forward_kinematics(q, L)
%FORWARD_KINEMATICS Tool and elbow positions in the base frame {B}.
%   [P_tool, P_elbow] = FORWARD_KINEMATICS(q, L) for joint angles q (3xN,
%   one configuration per column) and link lengths L = [L1; L2].
%   T_BT is the base to tool transform built from the DH chain, for the
%   first column of q only.
%
%   Modified DH parameters (Craig):
%     i   alpha_{i-1}   a_{i-1}   d_i   theta_i
%     1   0             0         0     q1      shoulder azimuth
%     2   90 deg        0         0     q2      shoulder elevation
%     3   0             L1        0     q3      elbow
%     T   0             L2        0     0       tool

c1 = cos(q(1,:)); s1 = sin(q(1,:));
c2 = cos(q(2,:)); s2 = sin(q(2,:));
c23 = cos(q(2,:) + q(3,:)); s23 = sin(q(2,:) + q(3,:));

r_elbow = L(1)*c2;                       % horizontal distance from the shoulder axis
r_tool  = L(1)*c2 + L(2)*c23;
P_elbow = [r_elbow.*c1; r_elbow.*s1; L(1)*s2];
P_tool  = [r_tool.*c1;  r_tool.*s1;  L(1)*s2 + L(2)*s23];

if nargout > 2
    T_BT = dh_transform(0,    0,    0, q(1,1)) * ...
           dh_transform(pi/2, 0,    0, q(2,1)) * ...
           dh_transform(0,    L(1), 0, q(3,1)) * ...
           dh_transform(0,    L(2), 0, 0);
end
end
