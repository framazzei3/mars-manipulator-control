function T = dh_transform(alpha, a, d, theta)
%DH_TRANSFORM Link transform from modified Denavit-Hartenberg parameters.
%   T = DH_TRANSFORM(alpha, a, d, theta) is the transform of frame {i}
%   relative to {i-1} with Craig's convention:
%   T = Rot_x(alpha_{i-1}) Trans_x(a_{i-1}) Rot_z(theta_i) Trans_z(d_i).
%   Angles in radians.

ca = cos(alpha); sa = sin(alpha);
ct = cos(theta); st = sin(theta);
T = [ ct,     -st,      0,    a
      st*ca,   ct*ca,  -sa,  -sa*d
      st*sa,   ct*sa,   ca,   ca*d
      0,       0,       0,    1 ];
end
