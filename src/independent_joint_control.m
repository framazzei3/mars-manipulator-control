function controller = independent_joint_control(p, ref)
%INDEPENDENT_JOINT_CONTROL Linear control, one decoupled loop per joint.
%   controller = INDEPENDENT_JOINT_CONTROL(p, ref) returns tau(t, q, qd).
%   Each joint is a unit mass after the partitioning
%     tau = alpha (qdd_d + kd ed + kp e) + beta qd
%   with alpha the largest effective inertia of each joint (diagonal of the
%   mass matrix, motor included, with the arm stretched out, q2 = q3 = 0)
%   and beta = eta^2 Bm the viscous friction. Gravity, coupling and
%   Coulomb friction are left to the feedback: gravity leaves a steady
%   error of about G / (alpha kp).

[~, M0] = inverse_dynamics(zeros(3, 1), zeros(3, 1), zeros(3, 1), p);
alpha = diag(diag(M0));
beta  = diag(p.eta.^2 * p.Bm);
controller = @control;

    function tau = control(t, q, qd)
        [q_des, qd_des, qdd_des] = ref(t);
        tau = alpha*(qdd_des + p.kd*(qd_des - qd) + p.kp*(q_des - q)) + beta*qd;
    end
end
