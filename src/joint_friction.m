function F = joint_friction(qd, p)
%JOINT_FRICTION Motor friction seen at the joints, through the gearboxes.
%   F = JOINT_FRICTION(qd, p) for joint rates qd (3x1). The motor turns at
%   eta*qd, and its friction torque is multiplied by eta at the joint:
%   viscous eta^2 Bm qd, Coulomb eta tau_c with the value for the sign of
%   the velocity.
%
%   The Coulomb step is smoothed with tanh over p.coulomb_width: above a
%   few times that rate it is the usual step, while at rest it does not
%   switch back and forth at every integration step (the arm stops at the
%   via points, and an exact step there makes the integrator stall).

w = tanh(qd / p.coulomb_width);
coulomb = p.tau_c(1)*max(w, 0) - p.tau_c(2)*min(w, 0);
F = p.eta.^2 * p.Bm .* qd + p.eta .* coulomb;
end
