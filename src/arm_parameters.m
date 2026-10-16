function p = arm_parameters()
%ARM_PARAMETERS Problem data and design choices, in one place.
%   SI units and radians unless stated otherwise. Problem data come from
%   the assignment (docs/assignment.pdf); trajectory, control and
%   simulation settings are design choices.

%% Links (assignment, Table 1): hollow cylinders, carbon fibre and titanium
len       = [1; 0.8];          % [m]
radius    = [0.07; 0.07];      % outer radius [m]
thickness = [0.001; 0.001];    % [m]
density   = [2000; 2000];      % [kg/m^3]
p.links = link_properties(len, radius, thickness, density);

% Mismodelled lengths (Table 5), same section and material: the controller
% of the last test uses these, the simulated arm keeps the real ones
p.links_model = link_properties([1.15; 0.73], radius, thickness, density);

p.g = 3.71;                    % Mars surface gravity [m/s^2]

%% Joint actuators (Table 2), joints 1 to 3
p.tau_max = [47; 148; 87];     % maximum joint torque [N m]
p.eta     = [125; 100; 100];   % gear ratio
p.Im      = 2e-4;              % rotor inertia [kg m^2]
p.Bm      = 8.17e-4;           % motor viscous friction [N m s/rad]
p.tau_c   = [1.26e-4; -7.09e-4];   % motor Coulomb friction, positive and negative velocity [N m]
p.coulomb_width = 1e-3;        % joint rate over which Coulomb friction changes sign [rad/s] (design)
p.q_min   = deg2rad([-135; -138; -120]);
p.q_max   = deg2rad([ 150;   80;  120]);

%% Frames: {B} at the shoulder, 1 m above the ground; {S} on the ground
% P_B = R_BS * P_S + p_BS
p.R_BS = [0 -1 0; 1 0 0; 0 0 1];   % rotation of 90 deg about z
p.p_BS = [0.5; 0.5; -1];           % origin of {S} in {B} [m]

%% Task (Tables 3 and 4), positions of the tool in {S} [m]
p.P_pick_S  = [-1.478;  1.185; 1.342];   % instrument on the lander deck
p.P_place_S = [-0.251; -0.430; 0.340];   % instrument on the ground
p.P_wrong_S = [-1.197;  1.340; 1.686];   % erroneous initial position (task 9)
p.workspace_height = 0.35;               % workspace section above the ground (task 3)

%% Trajectory (design)
p.tf = 10;                     % duration [s] (assignment)
p.dt = 0.1;                    % path update step [s] (assignment)
p.lift = 0.4;                  % via point 1: tool lifted straight up from the deck [m]
% Via point 2: same arm shape as via point 1, already at the final azimuth
p.segment_times = [2 5 3];     % pick to via 1, via 1 to via 2, via 2 to place [s]
p.blends_direct = [1 1];       % blend durations without via points [s]
p.blends_via    = [0.3 0.6 0.6 0.3];   % blend durations at pick, via 1, via 2, place [s]

%% Control (design)
% Critically damped error dynamics, natural frequency sqrt(kp) = 22.4 rad/s
p.kp = 500;
p.kd = 2*sqrt(p.kp);

%% Simulation
p.sim_dt = 0.01;               % output step of the closed loop simulations [s]
p.ode_options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.01);
end
