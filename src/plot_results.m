function plot_results(p, traj, sims, ctrl_names, path_points, ws_x, ws_y, ws_reach, out_dir)
%PLOT_RESULTS Figures of all tasks, saved as PNG in out_dir.

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
colors = lines(4);
joint_labels = {'Joint 1, azimuth', 'Joint 2, elevation', 'Joint 3, elbow'};
main_traj = numel(traj);        % parabolic blends with via points

%% The arm through the pick and place manoeuvre, in {S}
f = figure('Color', 'w', 'Position', [100 100 1000 760]);
hold on
[gx, gy] = meshgrid([-2.4 1.2], [-1.4 2.2]);
surf(gx, gy, zeros(size(gx)), 'FaceColor', [0.85 0.62 0.48], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
shoulder_S = base_to_station([0; 0; 0], p);
plot3(shoulder_S([1 1]), shoulder_S([2 2]), [0 shoulder_S(3)], '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 8);
names = {'Pick', 'Via point 1', 'Via point 2', 'Place'};
h = gobjects(1, 6);
for k = 1:4
    [P_tool, P_elbow] = forward_kinematics(path_points(:, k), p.links.L);
    P = base_to_station([[0; 0; 0], P_elbow, P_tool], p);
    h(k) = plot3(P(1, :), P(2, :), P(3, :), '-o', 'Color', colors(k, :), 'LineWidth', 3, ...
        'MarkerSize', 6, 'MarkerFaceColor', colors(k, :));
end
P = traj(main_traj).tool_S;
h(5) = plot3(P(1, :), P(2, :), P(3, :), 'k-', 'LineWidth', 1.5);
P = traj(1).tool_S;
h(6) = plot3(P(1, :), P(2, :), P(3, :), 'k:', 'LineWidth', 1.5);
grid on; axis equal; view(-50, 22)
xlabel('x_S [m]'); ylabel('y_S [m]'); zlabel('z_S [m]')
zlim([0 2])
legend(h, [names, {'Tool path, via points', 'Tool path, direct'}], 'Location', 'eastoutside')
save_figure(f, fullfile(out_dir, 'arm_pick_and_place.png'));

%% Workspace section
f = figure('Color', 'w', 'Position', [100 100 760 680]);
imagesc(ws_x(1, :), ws_y(:, 1), double(ws_reach));
colormap([1 1 1; 0.80 0.33 0.25]);
set(gca, 'YDir', 'normal');
hold on
plot(shoulder_S(1), shoulder_S(2), 'k+', 'MarkerSize', 12, 'LineWidth', 2);
plot(p.P_place_S(1), p.P_place_S(2), 'ko', 'MarkerSize', 9, 'LineWidth', 1.5);
axis equal; grid on; box on
axis([-2.4 1.4 -1.4 2.4])
xlabel('x_S [m]'); ylabel('y_S [m]')
legend({'Shoulder axis', 'Place position (34 cm)'}, 'Location', 'southwest')
title(sprintf('Workspace %.0f cm above the ground', 100*p.workspace_height), 'FontWeight', 'normal')
save_figure(f, fullfile(out_dir, 'workspace.png'));

%% Joint trajectories
f = figure('Color', 'w', 'Position', [100 100 1600 900]);
tl = tiledlayout(3, numel(traj), 'TileSpacing', 'compact', 'Padding', 'compact');
t_fine = 0:0.005:p.tf;
ylabels = {'Angle [deg]', 'Rate [deg/s]', 'Acceleration [deg/s^2]'};
for i = 1:numel(traj)
    [q, qd, qdd] = traj(i).ref(t_fine);
    data = {q, qd, qdd};
    for r = 1:3
        nexttile(tl, (r - 1)*numel(traj) + i);
        plot(t_fine, rad2deg(data{r}), 'LineWidth', 1.5); grid on
        if r == 1, title(traj(i).name, 'FontWeight', 'normal'); end
        if r == 3, xlabel('Time [s]'); end
        if i == 1, ylabel(ylabels{r}); end
        if r == 1 && i == 1, legend('\theta_1', '\theta_2', '\theta_3', 'Location', 'best'); end
    end
end
save_figure(f, fullfile(out_dir, 'trajectories.png'));

%% Torques to follow the trajectories, open loop
f = figure('Color', 'w', 'Position', [100 100 1100 820]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:3
    nexttile(tl);
    hold on
    for i = 1:numel(traj)
        plot(traj(i).t_fine, traj(i).tau(j, :), 'LineWidth', 1.5, 'Color', colors(i, :));
    end
    grid on; box on
    ylabel('\tau [N m]'); title(joint_labels{j}, 'FontWeight', 'normal')
    if j == 1, legend({traj.name}, 'Location', 'eastoutside'); end
    if j == 3, xlabel('Time [s]'); end
end
save_figure(f, fullfile(out_dir, 'open_loop_torques.png'));

%% Control errors, from the erroneous start
f = figure('Color', 'w', 'Position', [100 100 1400 520]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl);
for c = 1:numel(ctrl_names)
    s = sims{main_traj, c};
    semilogy(s.t, max(rad2deg(vecnorm(s.e)), 1e-12), 'LineWidth', 1.5, 'Color', colors(c, :)); hold on
end
grid on
xlabel('Time [s]'); ylabel('Joint angle error, norm [deg]')
legend(ctrl_names, 'Location', 'northeast')
ylim([1e-10 1e2])
title(traj(main_traj).name, 'FontWeight', 'normal')
nexttile(tl);
styles = {'-', '-', '--'};
hold on
hj = gobjects(1, 3);
for c = [1 3]
    s = sims{main_traj, c};
    for j = 1:3
        hl = plot(s.t, rad2deg(s.e(j, :)), styles{c}, 'LineWidth', 1.5, 'Color', colors(j, :));
        if c == 1, hj(j) = hl; end
    end
end
grid on; box on
xlim([1 p.tf])
xlabel('Time [s]'); ylabel('Joint angle error [deg]')
legend(hj, {'\theta_1', '\theta_2', '\theta_3'}, 'Location', 'best')
title('After the transient: linear (solid), mismodelled (dashed)', 'FontWeight', 'normal')
save_figure(f, fullfile(out_dir, 'control_errors.png'));

%% Control torques
f = figure('Color', 'w', 'Position', [100 100 1400 820]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
windows = {[0 1], [1 p.tf]};
for j = 1:3
    for w = 1:2
        nexttile(tl);
        hold on
        hc = gobjects(1, numel(ctrl_names) + 1);
        for c = 1:numel(ctrl_names)
            s = sims{main_traj, c};
            hc(c) = plot(s.t, s.tau(j, :), 'LineWidth', 1.5, 'Color', colors(c, :));
        end
        hc(end) = plot(traj(main_traj).t_fine, traj(main_traj).tau(j, :), 'k--', 'LineWidth', 1.2);
        if w == 1
            yline([-1 1]*p.tau_max(j), 'k:');
            ylim([-1.1 1.1]*p.tau_max(j))
            ylabel('\tau [N m]')
        end
        xlim(windows{w}); grid on; box on
        title(sprintf('%s, %g to %g s', joint_labels{j}, windows{w}), 'FontWeight', 'normal')
        if j == 3, xlabel('Time [s]'); end
        if j == 1 && w == 2
            legend(hc, [ctrl_names, {'Open loop, nominal'}], 'Location', 'eastoutside');
        end
    end
end
save_figure(f, fullfile(out_dir, 'control_torques.png'));
end

function save_figure(f, file)
if isprop(f, 'Theme')
    f.Theme = 'light';      % newer MATLAB releases may default to dark
end
exportgraphics(f, file, 'Resolution', 200);
close(f);
end
