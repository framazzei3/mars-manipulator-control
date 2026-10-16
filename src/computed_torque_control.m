function controller = computed_torque_control(p, ref, links)
%COMPUTED_TORQUE_CONTROL Nonlinear control in joint space (computed torque).
%   controller = COMPUTED_TORQUE_CONTROL(p, ref, links) returns
%   tau(t, q, qd) = M(q) (qdd_d + kd ed + kp e) + V(q, qd) + G(q) + F(qd),
%   with the model of the arm built from links: p.links for perfect
%   knowledge, p.links_model for the mismodelled arm. With a perfect model
%   the closed loop is e'' + kd e' + kp e = 0 for every joint, decoupled.

controller = @control;

    function tau = control(t, q, qd)
        [q_des, qd_des, qdd_des] = ref(t);
        tau = inverse_dynamics(q, qd, qdd_des + p.kd*(qd_des - qd) + p.kp*(q_des - q), p, links);
    end
end
