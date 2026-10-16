function links = link_properties(L, r, s, rho)
%LINK_PROPERTIES Mass and inertia of links modelled as hollow cylinders.
%   links = LINK_PROPERTIES(L, r, s, rho) with column vectors of length L,
%   outer radius r, wall thickness s and density rho, one row per link.
%   Each link lies along the x axis of its frame, so Ixx is the axial
%   moment of inertia and Iyy = Izz the transverse one, about the centre
%   of mass.

ri = r - s;                                   % inner radius
links.L   = L;
links.m   = rho .* pi .* (r.^2 - ri.^2) .* L;
links.Ixx = links.m .* (r.^2 + ri.^2) / 2;
links.Iyy = links.m .* (3*(r.^2 + ri.^2) + L.^2) / 12;
links.Izz = links.Iyy;
end
