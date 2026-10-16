%CHECK_MODEL Independent checks of the kinematic and dynamic model.
%   1. Closed form forward kinematics against the DH chain.
%   2. Inverse kinematics round trip on random configurations.
%   3. Mass matrix, Coriolis and gravity terms of src/arm_dynamics.m
%      against a Lagrangian formulation built numerically from the DH
%      frames (link Jacobians, Christoffel symbols, potential energy).
%   4. Energy conservation of the free, frictionless arm.
%   Prints the largest discrepancy of each check. Needs base MATLAB only.

clear; clc
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
p = arm_parameters();
rng(1, 'twister');
n_samples = 200;
random_q = @() p.q_min + (p.q_max - p.q_min) .* rand(3, 1);

%% 1. Forward kinematics
err_fk = 0;
for k = 1:n_samples
    q = random_q();
    [P, ~, T] = forward_kinematics(q, p.links.L);
    err_fk = max(err_fk, norm(T(1:3, 4) - P));
end

%% 2. Inverse kinematics round trip
err_ik = 0;
for k = 1:n_samples
    q = random_q();
    [Q, ~, ok] = inverse_kinematics(forward_kinematics(q, p.links.L), p);
    % Every returned solution must reach the point, and one must be q
    P = forward_kinematics(q, p.links.L);
    for j = 1:4
        err_ik = max(err_ik, norm(forward_kinematics(Q(:, j), p.links.L) - P));
    end
    err_ik = max(err_ik, min(vecnorm(angle_difference(Q(:, ok), q))));
end

%% 3. Dynamics against a numerical Lagrangian model
err_M = 0; err_V = 0; err_G = 0;
for k = 1:n_samples
    q = random_q();
    qd = 2*rand(3, 1) - 1;
    [M, V, G] = arm_dynamics(q, qd, p.links, p.g);
    [M_ref, V_ref, G_ref] = lagrangian_dynamics(q, qd, p.links, p.g);
    err_M = max(err_M, norm(M - M_ref) / norm(M_ref));
    err_V = max(err_V, norm(V - V_ref) / max(norm(V_ref), 1e-3));
    err_G = max(err_G, norm(G - G_ref) / norm(G_ref));
end

%% 4. Energy of the free arm, no friction, no torque
energy = @(x) 0.5*x(4:6).'*arm_dynamics(x(1:3), x(4:6), p.links, p.g)*x(4:6) + potential_energy(x(1:3), p.links, p.g);
x0 = [0.3; 0.5; -1; 0.8; -0.5; 1.2];
[~, x] = ode45(@(t, x) free_arm(x, p), [0 10], x0, odeset('RelTol', 1e-11, 'AbsTol', 1e-12));
E = arrayfun(@(k) energy(x(k, :).'), 1:size(x, 1));
err_E = max(abs(E - E(1))) / abs(E(1));

fprintf('Forward kinematics, closed form vs DH chain     %9.2e m\n', err_fk);
fprintf('Inverse kinematics round trip                   %9.2e (m and rad)\n', err_ik);
fprintf('Mass matrix vs Lagrangian, relative             %9.2e\n', err_M);
fprintf('Coriolis and centrifugal vs Lagrangian, rel.    %9.2e\n', err_V);
fprintf('Gravity vs Lagrangian, relative                 %9.2e\n', err_G);
fprintf('Energy drift of the free arm over 10 s, rel.    %9.2e\n', err_E);

%% Local functions
function d = angle_difference(a, b)
d = mod(a - b + pi, 2*pi) - pi;
end

function dx = free_arm(x, p)
[M, V, G] = arm_dynamics(x(1:3), x(4:6), p.links, p.g);
dx = [x(4:6); -M \ (V + G)];
end

function [R, c] = link_frames(q, links)
% Rotation of the link frames ({2} for link 1, {3} for link 2) and
% position of the centres of mass, from the DH chain
T1 = dh_transform(0, 0, 0, q(1));
T2 = T1 * dh_transform(pi/2, 0, 0, q(2));
T3 = T2 * dh_transform(0, links.L(1), 0, q(3));
R = {T2(1:3, 1:3), T3(1:3, 1:3)};
c = [T2 * [links.L(1)/2; 0; 0; 1], T3 * [links.L(2)/2; 0; 0; 1]];
c = c(1:3, :);
R{3} = T1(1:3, 1:3);
end

function U = potential_energy(q, links, g)
[~, c] = link_frames(q, links);
U = g * (links.m(1)*c(3, 1) + links.m(2)*c(3, 2));
end

function M = lagrangian_mass(q, links)
% M = sum of m Jv' Jv + Jw' R I R' Jw over the two links
step = 1e-6;
[R, c] = link_frames(q, links);
Jv = zeros(3, 3, 2);
for j = 1:3
    dq = zeros(3, 1); dq(j) = step;
    [~, c_plus]  = link_frames(q + dq, links);
    [~, c_minus] = link_frames(q - dq, links);
    Jv(:, j, :) = reshape((c_plus - c_minus) / (2*step), 3, 1, 2);
end
z1 = R{3}(:, 3); z2 = R{1}(:, 3); z3 = R{2}(:, 3);
Jw = {[z1, z2, zeros(3, 1)], [z1, z2, z3]};
M = zeros(3);
for i = 1:2
    I = R{i} * diag([links.Ixx(i), links.Iyy(i), links.Izz(i)]) * R{i}.';
    M = M + links.m(i) * Jv(:, :, i).' * Jv(:, :, i) + Jw{i}.' * I * Jw{i};
end
end

function [M, V, G] = lagrangian_dynamics(q, qd, links, g)
step = 1e-4;
M = lagrangian_mass(q, links);
dM = zeros(3, 3, 3);       % dM(:,:,k) = dM/dq_k
G = zeros(3, 1);
for k = 1:3
    dq = zeros(3, 1); dq(k) = step;
    dM(:, :, k) = (lagrangian_mass(q + dq, links) - lagrangian_mass(q - dq, links)) / (2*step);
    G(k) = (potential_energy(q + dq, links, g) - potential_energy(q - dq, links, g)) / (2*step);
end
% Christoffel symbols: V_i = sum_jk (dM_ij/dq_k - 1/2 dM_jk/dq_i) qd_j qd_k
V = zeros(3, 1);
for i = 1:3
    for j = 1:3
        for k = 1:3
            V(i) = V(i) + (dM(i, j, k) - 0.5*dM(j, k, i)) * qd(j) * qd(k);
        end
    end
end
end
