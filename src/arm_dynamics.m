function [M, V, G] = arm_dynamics(q, qd, links, g)
%ARM_DYNAMICS Rigid body dynamics of the arm: tau = M(q) qdd + V(q, qd) + G(q).
%   [M, V, G] = ARM_DYNAMICS(q, qd, links, g) for one configuration q and
%   joint rates qd (3x1), link properties from LINK_PROPERTIES and gravity
%   g along -z of {B}. Motors and friction are added by INVERSE_DYNAMICS.
%
%   Closed form of the Newton-Euler equations (derivation in the README,
%   checked against an independent Lagrangian formulation in
%   check_model.m). Link i has its centre of mass at L_i/2.
%   Azimuth inertia: M11 = sum of transverse and axial inertias projected
%   on the vertical, plus m r^2 of each centre of mass at horizontal
%   distance r from the azimuth axis. The vertical plane of the arm is a
%   plane of symmetry, so M12 = M13 = 0.

L1 = links.L(1);    L2 = links.L(2);
m1 = links.m(1);    m2 = links.m(2);
Ia1 = links.Ixx(1); Ia2 = links.Ixx(2);    % axial
It1 = links.Iyy(1); It2 = links.Iyy(2);    % transverse (Iyy = Izz)

q2 = q(2); q3 = q(3);
c2 = cos(q2); s2 = sin(q2);
c3 = cos(q3); s3 = sin(q3);
c23 = cos(q2 + q3); s23 = sin(q2 + q3);

% Horizontal distance of the centres of mass from the azimuth axis
r1 = L1/2*c2;
r2 = L1*c2 + L2/2*c23;

%% Mass matrix
M11 = It1*c2^2 + Ia1*s2^2 + m1*r1^2 + It2*c23^2 + Ia2*s23^2 + m2*r2^2;
M22 = links.Izz(1) + links.Izz(2) + m1*L1^2/4 + m2*(L1^2 + L2^2/4 + L1*L2*c3);
M23 = links.Izz(2) + m2*(L2^2/4 + L1*L2/2*c3);
M33 = links.Izz(2) + m2*L2^2/4;
M = [M11 0 0; 0 M22 M23; 0 M23 M33];

%% Coriolis and centrifugal terms
% Derivatives of M11 with respect to q2 and q3
dr1_2 = -L1/2*s2;
dr2_2 = -L1*s2 - L2/2*s23;
dr2_3 = -L2/2*s23;
dM11_2 = 2*(Ia1 - It1)*s2*c2 + 2*(Ia2 - It2)*s23*c23 + 2*m1*r1*dr1_2 + 2*m2*r2*dr2_2;
dM11_3 = 2*(Ia2 - It2)*s23*c23 + 2*m2*r2*dr2_3;
h = m2*L1*L2/2*s3;                          % elbow coupling, -dM23/dq3

V = [ dM11_2*qd(1)*qd(2) + dM11_3*qd(1)*qd(3)
     -dM11_2/2*qd(1)^2 - h*(2*qd(2)*qd(3) + qd(3)^2)
     -dM11_3/2*qd(1)^2 + h*qd(2)^2 ];

%% Gravity
G = [0
     g*((m1*L1/2 + m2*L1)*c2 + m2*L2/2*c23)
     g*m2*L2/2*c23];
end
