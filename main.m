%MAIN Kinematics, trajectories, dynamics and control of a Mars arm.
%   Simplified model of the InSight Instrument Deployment Arm: three
%   revolute joints, picking an instrument from the lander deck and placing
%   it on the ground in 10 s.
%   Tasks 1-3: kinematics and workspace section 35 cm above the ground.
%   Tasks 4-5: inverse kinematics and four joint trajectories.
%   Tasks 6-7: dynamics and the torques the motors must deliver.
%   Tasks 8-10: linear and computed torque control from an erroneous
%               initial pose, with perfect and mismodelled links.
%
%   Run from this folder. Figures go to results/figures and the numbers to
%   results/summary.txt. Needs base MATLAB only. check_model.m verifies
%   the kinematic and dynamic model independently.

clear; close all; clc
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));
p = arm_parameters();

%% Inverse kinematics (task 4) and via points
q_pick  = elbow_up_solution(p.P_pick_S, p);
q_place = elbow_up_solution(p.P_place_S, p);
q_via1  = elbow_up_solution(p.P_pick_S + [0; 0; p.lift], p);
q_via2  = [q_place(1); q_via1(2:3)];     % rotate in azimuth with the arm raised
path_points = [q_pick, q_via1, q_via2, q_place];

% The erroneous initial position of task 9 needs an azimuth beyond the
% joint limit: it is kept as given, as the disturbance to recover from
[q_wrong, wrong_within_limits] = elbow_up_solution(p.P_wrong_S, p);

%% Workspace section (task 3)
[ws_x, ws_y, ws_reach] = workspace_section(p, p.workspace_height, 3, 0.01);

%% Trajectories (task 5)
traj = struct('name', {}, 'ref', {});
traj(1).name = 'Quintic';
traj(1).ref  = quintic_trajectory([q_pick, q_place], p.tf);
traj(2).name = 'Quintic, via points';
traj(2).ref  = quintic_trajectory(path_points, p.segment_times);
traj(3).name = 'Parabolic blends';
traj(3).ref  = lspb_trajectory([q_pick, q_place], p.tf, p.blends_direct);
traj(4).name = 'Parabolic blends, via points';
traj(4).ref  = lspb_trajectory(path_points, p.segment_times, p.blends_via);

t = 0:p.dt:p.tf;
for i = 1:numel(traj)
    [traj(i).q, traj(i).qd, traj(i).qdd] = traj(i).ref(t);
    % On a fine time grid: torques for the motors (tasks 6-7), real arm,
    % and clearance from the ground of tool and elbow
    traj(i).t_fine = 0:0.01:p.tf;
    [q_fine, qd_fine, qdd_fine] = traj(i).ref(traj(i).t_fine);
    traj(i).tau = inverse_dynamics(q_fine, qd_fine, qdd_fine, p);
    [P_tool, P_elbow] = forward_kinematics(q_fine, p.links.L);
    P_elbow_S = base_to_station(P_elbow, p);
    traj(i).tool_S = base_to_station(P_tool, p);
    traj(i).min_tool_height  = min(traj(i).tool_S(3, :));
    traj(i).min_elbow_height = min(P_elbow_S(3, :));
    traj(i).within_limits = all(q_fine >= p.q_min & q_fine <= p.q_max, 'all');
end

%% Control (tasks 8-10), from the erroneous initial pose
ctrl_names = {'Linear, independent joints', 'Computed torque', 'Computed torque, mismodelled links'};
sims = cell(numel(traj), numel(ctrl_names));
for i = 1:numel(traj)
    ref = traj(i).ref;
    controllers = {independent_joint_control(p, ref), ...
                   computed_torque_control(p, ref, p.links), ...
                   computed_torque_control(p, ref, p.links_model)};
    for c = 1:numel(controllers)
        sims{i, c} = simulate_control(p, ref, controllers{c}, q_wrong);
    end
end

%% Figures
plot_results(p, traj, sims, ctrl_names, path_points, ws_x, ws_y, ws_reach, ...
    fullfile(root, 'results', 'figures'));

%% Summary
deg = @(x) rad2deg(x);
steady = @(s) max(abs(s.e(:, s.t >= 2)), [], 2);           % largest error after 2 s
settle = @(s) settling_time(s, deg2rad(1));                      % error below 1 deg from then on
saturated = @(s) s.t(find(any(abs(s.tau) >= p.tau_max - 1e-9, 1), 1, 'last'));

lines = {
    'Links'
    sprintf('  Masses                         %s kg', mat2str(p.links.m.', 4))
    sprintf('  Mismodelled masses             %s kg', mat2str(p.links_model.m.', 4))
    'Inverse kinematics, elbow up [deg]'
    sprintf('  Pick                           %s', mat2str(deg(q_pick.'), 5))
    sprintf('  Via point 1 (lifted %.1f m)     %s', p.lift, mat2str(deg(q_via1.'), 5))
    sprintf('  Via point 2                    %s', mat2str(deg(q_via2.'), 5))
    sprintf('  Place                          %s', mat2str(deg(q_place.'), 5))
    sprintf('  Erroneous start                %s (within limits: %d)', mat2str(deg(q_wrong.'), 5), wrong_within_limits)
    sprintf('  Initial error                  %s', mat2str(deg((q_pick - q_wrong).'), 3))
    'Trajectories'
    };
for i = 1:numel(traj)
    lines{end+1} = sprintf('  %-30s max |tau| %s N m, min tool height %.3f m, min elbow height %.3f m, joint limits respected %d', ...
        traj(i).name, mat2str(max(abs(traj(i).tau), [], 2).', 3), traj(i).min_tool_height, traj(i).min_elbow_height, traj(i).within_limits); %#ok<SAGROW>
end
lines{end+1} = 'Control, from the erroneous start';
for i = 1:numel(traj)
    lines{end+1} = sprintf('  %s', traj(i).name); %#ok<SAGROW>
    for c = 1:numel(ctrl_names)
        s = sims{i, c};
        lines{end+1} = sprintf('    %-36s error after 2 s %s deg, below 1 deg after %.2f s, saturated until %.2f s', ...
            ctrl_names{c}, mat2str(deg(steady(s)).', 3), settle(s), saturated(s)); %#ok<SAGROW>
    end
end
[~, M0] = inverse_dynamics(zeros(3, 1), zeros(3, 1), zeros(3, 1), p);
G_place = inverse_dynamics(q_place, zeros(3, 1), zeros(3, 1), p);
lines{end+1} = sprintf('Linear control: expected gravity droop G/(alpha kp) at the place pose %s deg', ...
    mat2str(deg(G_place ./ (diag(M0) * p.kp)).', 3));

summary = strjoin(lines, newline);
disp(summary)
fid = fopen(fullfile(root, 'results', 'summary.txt'), 'w');
fprintf(fid, '%s\n', summary);
fclose(fid);

function t_settle = settling_time(s, threshold)
% First time after which the joint error norm stays below threshold
k = find(vecnorm(s.e) > threshold, 1, 'last');
if isempty(k)
    t_settle = 0;
elseif k == numel(s.t)
    t_settle = Inf;
else
    t_settle = s.t(k + 1);
end
end
