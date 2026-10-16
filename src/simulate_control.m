function sim = simulate_control(p, ref, controller, q_init)
%SIMULATE_CONTROL Closed loop simulation of the arm following a trajectory.
%   sim = SIMULATE_CONTROL(p, ref, controller, q_init) integrates the real
%   arm (p.links) from rest at q_init, with joint torques
%   controller(t, q, qd) saturated at p.tau_max, over [0, p.tf].
%   ref(t) gives the desired position, velocity and acceleration.
%
%   sim.t, sim.q, sim.qd, sim.tau: time, state and applied torques
%   sim.e, sim.ed: joint angle and rate errors (desired minus actual)

t_out = 0:p.sim_dt:p.tf;
x0 = [q_init; zeros(3, 1)];
[t, x] = ode45(@(t, x) arm_state_derivative(t, x, p, controller), t_out, x0, p.ode_options);

sim.t  = t.';
sim.q  = x(:, 1:3).';
sim.qd = x(:, 4:6).';
sim.tau = zeros(3, numel(t));
for k = 1:numel(t)
    sim.tau(:,k) = saturate(controller(t(k), sim.q(:,k), sim.qd(:,k)), p.tau_max);
end
[q_des, qd_des] = ref(sim.t);
sim.e  = q_des - sim.q;
sim.ed = qd_des - sim.qd;
end

function dx = arm_state_derivative(t, x, p, controller)
q = x(1:3); qd = x(4:6);
tau = saturate(controller(t, q, qd), p.tau_max);
[~, M, h] = inverse_dynamics(q, qd, zeros(3, 1), p);
dx = [qd; M \ (tau - h)];
end

function tau = saturate(tau, tau_max)
tau = max(min(tau, tau_max), -tau_max);
end
