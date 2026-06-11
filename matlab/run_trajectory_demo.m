clear; clc; close all;

workspace_root = fileparts(fileparts(mfilename('fullpath')));
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');
output_root = fullfile(workspace_root, 'outputs');

[robot, config, mdh_table] = build_robot_6dof(config_path);

q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_goal_deg = reshape(double(config.default_state.q_goal_deg), 1, []);
q_init = deg2rad(q_init_deg);
q_goal = deg2rad(q_goal_deg);

duration_s = double(config.simulation_defaults.duration_s);
steps = double(config.simulation_defaults.steps);
t = linspace(0, duration_s, steps).';

disp('================ MDH Table ================');
disp(mdh_table);

%% 1) Point-to-point joint trajectory
q_ptp = make_ptp_trajectory(q_init, q_goal, t);
ptp_result = evaluate_trajectory(robot, q_ptp, t, [], 'matlab', 'ptp');
export_simulation_results(fullfile(output_root, 'matlab_ptp'), ptp_result);

%% 2) Cartesian circular tracking in a plane
[q_track, desired_positions] = make_cartesian_track(robot, config, q_init, t);
track_result = evaluate_trajectory(robot, q_track, t, desired_positions, 'matlab', 'cartesian_track');
export_simulation_results(fullfile(output_root, 'matlab_cartesian_track'), track_result);

%% 3) Visualizations
plot_joint_trajectory(t, ptp_result.q_deg, 'Point-to-point joint motion');
plot_cartesian_track(desired_positions, track_result.position_m, 'Cartesian tracking result');
animate_robot_motion(robot, q_ptp, ptp_result.position_m, 'PTP robot motion', [0.85 0.15 0.15]);
animate_robot_motion(robot, q_track, track_result.position_m, 'Cartesian tracking robot motion', [0.15 0.35 0.85]);

fprintf('\nMATLAB trajectory demo finished.\n');
fprintf('PTP output folder: %s\n', fullfile(output_root, 'matlab_ptp'));
fprintf('Cartesian tracking output folder: %s\n', fullfile(output_root, 'matlab_cartesian_track'));

function q_traj = make_ptp_trajectory(q0, qf, t)
q0 = reshape(q0, 1, []);
qf = reshape(qf, 1, []);

if exist('jtraj', 'file') == 2
    q_traj = jtraj(q0, qf, t);
    return;
end

tau = (t - t(1)) / max(t(end) - t(1), eps);
s = 10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5;
q_traj = q0 + (qf - q0) .* s;
end

function [q_track, desired_positions] = make_cartesian_track(robot, config, q_start, t)
num_samples = numel(t);
num_joints = robot.n;
T_start = robot.fkine(q_start).T;
start_position = T_start(1:3, 4).';
base_radius = double(config.simulation_defaults.cartesian_track_radius_m);
center_hint = reshape(double(config.simulation_defaults.cartesian_track_center_offset_m), 1, 3);
plane_name = lower(string(config.simulation_defaults.cartesian_track_plane));

radius_candidates = [base_radius, 0.8 * base_radius, 0.6 * base_radius, 0.4 * base_radius];
best_q_track = [];
best_desired_positions = [];
best_max_error = inf;

for candidate_idx = 1:numel(radius_candidates)
    radius = max(radius_candidates(candidate_idx), 0.005);
    [center, radial_dir, tangent_dir] = circle_frame_from_hint(start_position, center_hint, radius, plane_name);

    desired_positions_candidate = zeros(num_samples, 3);
    for k = 1:num_samples
        phase = 2 * pi * (k - 1) / max(num_samples - 1, 1);
        desired_positions_candidate(k, :) = circle_position(center, radius, phase, radial_dir, tangent_dir);
    end

    q_track_candidate = track_circle_position_only(robot, q_start, desired_positions_candidate);
    actual_positions_candidate = fk_positions(robot, q_track_candidate);
    error_norm = vecnorm(actual_positions_candidate - desired_positions_candidate, 2, 2);
    max_error = max(error_norm);

    if max_error < best_max_error
        best_max_error = max_error;
        best_q_track = q_track_candidate;
        best_desired_positions = desired_positions_candidate;
    end

    if max_error < 5e-3
        q_track = q_track_candidate;
        desired_positions = desired_positions_candidate;
        return;
    end
end

warning('RTB:CircleTrackApprox', ...
    ['Circle tracking stayed approximate. Using best candidate with max position error %.4f m. ', ...
     'Consider reducing radius or changing initial pose for a tighter circle.'], ...
    best_max_error);
q_track = best_q_track;
desired_positions = best_desired_positions;
end

function p = circle_position(center, radius, phase, radial_dir, tangent_dir)
p = center + radius * (radial_dir * cos(phase) + tangent_dir * sin(phase));
end

function [center, radial_dir, tangent_dir] = circle_frame_from_hint(start_position, center_hint, radius, plane_name)
[radial_dir, tangent_dir] = plane_unit_vectors(plane_name, center_hint);
center = start_position - radius * radial_dir;
end

function [radial_dir, tangent_dir] = plane_unit_vectors(plane_name, center_hint)
hint = zeros(1, 3);

switch plane_name
    case "yz"
        hint(2:3) = center_hint(2:3);
        if norm(hint(2:3)) < 1e-9
            hint = [0 1 0];
        end
        radial_dir = hint / norm(hint);
        tangent_dir = [0, -radial_dir(3), radial_dir(2)];
    case "xz"
        hint([1 3]) = center_hint([1 3]);
        if norm(hint([1 3])) < 1e-9
            hint = [1 0 0];
        end
        radial_dir = hint / norm(hint);
        tangent_dir = [-radial_dir(3), 0, radial_dir(1)];
    otherwise
        hint(1:2) = center_hint(1:2);
        if norm(hint(1:2)) < 1e-9
            hint = [1 0 0];
        end
        radial_dir = hint / norm(hint);
        tangent_dir = [-radial_dir(2), radial_dir(1), 0];
end

tangent_dir = tangent_dir / norm(tangent_dir);
end

function q_track = track_circle_position_only(robot, q_start, desired_positions)
num_samples = size(desired_positions, 1);
q_track = zeros(num_samples, robot.n);
q_track(1, :) = q_start;

for k = 2:num_samples
    q_curr = q_track(k - 1, :);
    q_track(k, :) = solve_position_step(robot, q_curr, desired_positions(k, :));
end
end

function q_next = solve_position_step(robot, q_seed, p_target)
q_next = reshape(q_seed, 1, []);
max_iterations = 35;
tolerance = 1e-4;
damping = 1e-3;
gain = 0.85;

for iter = 1:max_iterations
    T_curr = robot.fkine(q_next).T;
    pos_curr = T_curr(1:3, 4).';
    pos_err = p_target - pos_curr;
    if norm(pos_err) < tolerance
        break;
    end

    J0 = robot.jacob0(q_next);
    Jv = J0(1:3, :);
    dq = (Jv' / (Jv * Jv' + damping^2 * eye(3))) * pos_err(:);
    step_scale = min(1.0, 0.15 / max(norm(dq), eps));
    q_next = q_next + gain * step_scale * dq(:).';
    q_next = clamp_to_limits(robot, q_next);
end
end

function q_clamped = clamp_to_limits(robot, q_row)
q_clamped = reshape(q_row, 1, []);
for idx = 1:robot.n
    q_clamped(idx) = min(max(q_clamped(idx), robot.links(idx).qlim(1)), robot.links(idx).qlim(2));
end
end

function positions = fk_positions(robot, q_traj)
positions = zeros(size(q_traj, 1), 3);
for k = 1:size(q_traj, 1)
    T = robot.fkine(q_traj(k, :)).T;
    positions(k, :) = T(1:3, 4).';
end
end

function result = evaluate_trajectory(robot, q_traj, t, desired_positions, source_name, trajectory_type)
num_samples = size(q_traj, 1);
position_m = zeros(num_samples, 3);
rpy_deg = zeros(num_samples, 3);
transforms = zeros(4, 4, num_samples);

for k = 1:num_samples
    T = robot.fkine(q_traj(k, :)).T;
    transforms(:, :, k) = T;
    position_m(k, :) = T(1:3, 4).';
    rpy_deg(k, :) = pose_from_transform(T);
end

if isempty(desired_positions)
    position_error = zeros(num_samples, 1);
else
    position_error = vecnorm(position_m - desired_positions, 2, 2);
end

result = struct();
result.summary = struct( ...
    'robot_name', robot.name, ...
    'source', source_name, ...
    'trajectory_type', trajectory_type, ...
    'time_span_s', t(end) - t(1), ...
    'samples', num_samples);
result.time_s = t;
result.q_rad = q_traj;
result.q_deg = rad2deg(q_traj);
result.position_m = position_m;
result.rpy_deg = rpy_deg;
result.transforms = transforms;
result.metrics = struct( ...
    'max_position_error_m', max(position_error), ...
    'mean_position_error_m', mean(position_error));

if isempty(desired_positions)
    result.desired_position_m = position_m;
else
    result.desired_position_m = desired_positions;
end
end

function rpy_deg = pose_from_transform(T)
R = T(1:3, 1:3);
yaw = atan2(R(2,1), R(1,1));
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
roll = atan2(R(3,2), R(3,3));
rpy_deg = rad2deg([roll, pitch, yaw]);
end

function plot_joint_trajectory(t, q_deg, title_text)
figure('Name', title_text, 'Color', 'w');
for idx = 1:size(q_deg, 2)
    subplot(size(q_deg, 2), 1, idx);
    plot(t, q_deg(:, idx), 'LineWidth', 1.2);
    grid on;
    ylabel(sprintf('q%d (deg)', idx));
    if idx == 1
        title(title_text);
    end
end
xlabel('Time (s)');
end

function plot_cartesian_track(desired_positions, actual_positions, title_text)
figure('Name', title_text, 'Color', 'w');
plot3(desired_positions(:, 1), desired_positions(:, 2), desired_positions(:, 3), 'b--', 'LineWidth', 1.5);
hold on;
plot3(actual_positions(:, 1), actual_positions(:, 2), actual_positions(:, 3), 'r-', 'LineWidth', 1.5);
plot3(actual_positions(1, 1), actual_positions(1, 2), actual_positions(1, 3), 'go', 'MarkerFaceColor', 'g');
plot3(actual_positions(end, 1), actual_positions(end, 2), actual_positions(end, 3), 'ko', 'MarkerFaceColor', 'k');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(title_text);
legend('Desired path', 'Actual path', 'Start', 'End', 'Location', 'best');
end

function animate_robot_motion(robot, q_traj, tip_positions, figure_name, trail_color)
q_anim = q_traj(1:max(1, floor(size(q_traj, 1) / 60)):end, :);
tip_anim = tip_positions(1:max(1, floor(size(tip_positions, 1) / 60)):end, :);
workspace_bounds = compute_workspace_bounds(tip_positions);

try
    figure_handle = figure('Name', figure_name, 'Color', 'w');
    robot.plot(q_anim(1, :), ...
        'workspace', workspace_bounds, ...
        'trail', {trail_color, 'LineWidth', 1.5}, ...
        'fps', 20, ...
        'scale', 0.7, ...
        'jointdiam', 1.2);
    title(figure_name);
    drawnow;

    for k = 1:size(q_anim, 1)
        if ~ishandle(figure_handle)
            return;
        end
        robot.animate(q_anim(k, :));
        drawnow;
        pause(0.03);
    end
catch
    animate_robot_motion_fallback(robot, q_anim, tip_anim, figure_name, trail_color, workspace_bounds);
end
end

function animate_robot_motion_fallback(robot, q_traj, tip_positions, figure_name, trail_color, workspace_bounds)
figure('Name', [figure_name ' (fallback)'], 'Color', 'w');
axes_handle = axes();
hold(axes_handle, 'on');
grid(axes_handle, 'on');
axis(axes_handle, 'equal');
xlabel(axes_handle, 'X (m)');
ylabel(axes_handle, 'Y (m)');
zlabel(axes_handle, 'Z (m)');
title(axes_handle, figure_name);
xlim(axes_handle, workspace_bounds(1:2));
ylim(axes_handle, workspace_bounds(3:4));
zlim(axes_handle, workspace_bounds(5:6));

plot3(tip_positions(:, 1), tip_positions(:, 2), tip_positions(:, 3), '--', 'Color', trail_color, 'LineWidth', 1.0);
robot_line = plot3(0, 0, 0, '-o', 'LineWidth', 2.0, 'Color', [0.1 0.1 0.1], ...
    'MarkerFaceColor', [0.2 0.6 0.9], 'MarkerSize', 5);
tip_trace = animatedline('Color', trail_color, 'LineWidth', 1.8);

for k = 1:size(q_traj, 1)
    joint_positions = compute_joint_positions(robot, q_traj(k, :));
    set(robot_line, ...
        'XData', joint_positions(:, 1), ...
        'YData', joint_positions(:, 2), ...
        'ZData', joint_positions(:, 3));
    addpoints(tip_trace, joint_positions(end, 1), joint_positions(end, 2), joint_positions(end, 3));
    drawnow;
    pause(0.03);
end
end

function joint_positions = compute_joint_positions(robot, q_row)
q_row = reshape(q_row, 1, []);
joint_positions = zeros(robot.n + 2, 3);

T = transform_to_matrix(robot.base);
joint_positions(1, :) = T(1:3, 4).';

for idx = 1:robot.n
    A_idx = robot.links(idx).A(q_row(idx));
    T = T * transform_to_matrix(A_idx);
    joint_positions(idx + 1, :) = T(1:3, 4).';
end

tool_transform = T * transform_to_matrix(robot.tool);
joint_positions(end, :) = tool_transform(1:3, 4).';
end

function T = transform_to_matrix(transform_like)
if isempty(transform_like)
    T = eye(4);
    return;
end

if isnumeric(transform_like)
    T = double(transform_like);
elseif isobject(transform_like)
    try
        T = double(transform_like);
    catch
        try
            T = transform_like.T;
        catch
            error('Unsupported transform object type for animation.');
        end
    end
elseif isstruct(transform_like) && isfield(transform_like, 'T')
    T = double(transform_like.T);
else
    error('Unsupported transform value for animation.');
end

if ~isequal(size(T), [4 4])
    error('Expected a 4x4 homogeneous transform for animation.');
end
end

function workspace_bounds = compute_workspace_bounds(points_xyz)
mins = min(points_xyz, [], 1);
maxs = max(points_xyz, [], 1);
center = (mins + maxs) / 2;
radius = max(max(maxs - mins) / 2, 0.35);

workspace_bounds = [ ...
    center(1) - radius, center(1) + radius, ...
    center(2) - radius, center(2) + radius, ...
    center(3) - radius, center(3) + radius];
end
