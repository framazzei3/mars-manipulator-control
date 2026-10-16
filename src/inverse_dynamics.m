function [tau, M, h] = inverse_dynamics(q, qd, qdd, p, links)
%INVERSE_DYNAMICS Joint torques for given joint motion, motors included.
%   [tau, M, h] = INVERSE_DYNAMICS(q, qd, qdd, p, links) with
%   tau = M qdd + h, where M adds the rotor inertias reflected through the
%   gearboxes (eta^2 Im) to the link mass matrix and h = V + G + F.
%   q, qd and qdd may have several columns (one per time step); M and h
%   are then those of the last column. links defaults to p.links (the
%   real arm); pass p.links_model for the mismodelled one.

if nargin < 5
    links = p.links;
end
tau = zeros(size(q));
for k = 1:size(q, 2)
    [M, V, G] = arm_dynamics(q(:,k), qd(:,k), links, p.g);
    M = M + diag(p.eta.^2 * p.Im);
    h = V + G + joint_friction(qd(:,k), p);
    tau(:,k) = M*qdd(:,k) + h;
end
end
