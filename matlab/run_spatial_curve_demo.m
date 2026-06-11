clear; clc; close all;

workspace_root = fileparts(fileparts(mfilename('fullpath')));
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');
output_root = fullfile(workspace_root, 'outputs', 'matlab_spatial_curve_demo');

[robot, config, mdh_table] = build_robot_6dof(config_path);

q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_init = deg2rad(q_init_deg);
duration_s = double(config.simulation_defaults.duration_s);
steps = double(config.simulation_defaults.steps);
t = linspace(0, duration_s, steps).';

disp('================ Spatial Curve Demo MDH Table ================');
disp(mdh_table);

[q_traj, desired_positions, curve_info, singularity_metrics] = make_spatial_curve_track(robot, config, q_init, t);
result = evaluate_trajectory(robot, q_traj, t, desired_positions, 'matlab', 'spatial_curve_track');
result.metrics.max_condition_number = max(singularity_metrics.condition_numbers);
result.metrics.mean_condition_number = mean(singularity_metrics.condition_numbers);
result.metrics.path_length_m = path_length(result.position_m);
result.metrics.desired_path_length_m = path_length(desired_positions);
export_simulation_results(output_root, result);

fprintf('\nSpatial curve demo summary:\n');
fprintf('Curve type: 3D helix / spatial curve\n');
fprintf('IK method: %s\n', curve_info.ik_method);
fprintf('Requested radius = %.4f m\n', curve_info.requested_radius_m);
fprintf('Requested rise   = %.4f m\n', curve_info.rise_m);
fprintf('Turns            = %.2f\n', curve_info.turns);
fprintf('Axis direction   = [%.3f, %.3f, %.3f]\n', ...
    curve_info.axis_direction(1), curve_info.axis_direction(2), curve_info.axis_direction(3));
fprintf('Max position error  = %.6f m\n', result.metrics.max_position_error_m);
fprintf('Mean position error = %.6f m\n', result.metrics.mean_position_error_m);
fprintf('Max cond(J)         = %.3e\n', result.metrics.max_condition_number);
fprintf('Output folder: %s\n', output_root);

plot_spatial_curve(desired_positions, result.position_m, 'End-effector 3D spatial curve');
plot_joint_trajectory(t, result.q_deg, 'Spatial curve joint motion');
plot_position_components(t, desired_positions, result.position_m, 'Spatial curve Cartesian components');
plot_speed_profile(t, desired_positions, result.position_m, 'Spatial curve speed profile');
plot_tracking_error(t, desired_positions, result.position_m, 'Spatial curve tracking error');
plot_singularity_metric(t, singularity_metrics.condition_numbers, 'Spatial curve singularity metric');
animate_robot_motion(robot, q_traj, result.position_m, 'Spatial curve robot motion', [0.2 0.55 0.85]);

function [q_track, desired_positions, info, singularity_metrics] = make_spatial_curve_track(robot, config, q_start, t)
num_samples = numel(t);
T_start = robot.fkine(q_start).T;
start_position = T_start(1:3, 4).';

radius = double(config.simulation_defaults.spatial_curve_radius_m);
rise = double(config.simulation_defaults.spatial_curve_rise_m);
turns = double(config.simulation_defaults.spatial_curve_turns);
center_hint = reshape(double(config.simulation_defaults.spatial_curve_center_offset_m), 1, 3);
axis_direction = reshape(double(config.simulation_defaults.spatial_curve_axis_direction), 1, 3);

[curve_origin, basis_u, basis_v, axis_unit] = build_curve_frame(start_position, center_hint, axis_direction, radius);

desired_positions = zeros(num_samples, 3);
for k = 1:num_samples
    s = (k - 1) / max(num_samples - 1, 1);
    phase = 2 * pi * turns * s;
    axial_shift = rise * s;
    desired_positions(k, :) = curve_origin + ...
        radius * (basis_u * cos(phase) + basis_v * sin(phase)) + ...
        axial_shift * axis_unit;
end

[q_track, condition_numbers] = track_spatial_curve_inverse_kinematics(robot, q_start, desired_positions);
info = struct( ...
    'ik_method', 'ikine_q0_with_dls_fallback', ...
    'requested_radius_m', radius, ...
    'rise_m', rise, ...
    'turns', turns, ...
    'axis_direction', axis_unit);
singularity_metrics = struct('condition_numbers', condition_numbers);
end

function [curve_origin, basis_u, basis_v, axis_unit] = build_curve_frame(start_position, center_hint, axis_direction, radius)
axis_unit = reshape(axis_direction, 1, 3);
if norm(axis_unit) < 1e-9
    axis_unit = [1, 1, 1];
end
axis_unit = axis_unit / norm(axis_unit);

hint = center_hint - dot(center_hint, axis_unit) * axis_unit;
if norm(hint) < 1e-9
    arbitrary = [1, 0, 0];
    if abs(dot(arbitrary, axis_unit)) > 0.9
        arbitrary = [0, 1, 0];
    end
    hint = arbitrary - dot(arbitrary, axis_unit) * axis_unit;
end

basis_u = hint / norm(hint);
basis_v = cross(axis_unit, basis_u);
basis_v = basis_v / norm(basis_v);
curve_origin = start_position - radius * basis_u;
end

function [q_track, condition_numbers] = track_spatial_curve_inverse_kinematics(robot, q_start, desired_positions)
num_samples = size(desired_positions, 1);
q_track = zeros(num_samples, robot.n);
condition_numbers = zeros(num_samples, 1);
q_track(1, :) = q_start;
condition_numbers(1) = condition_number(robot, q_start);
T_start = robot.fkine(q_start).T;
R_target = T_start(1:3, 1:3);

for k = 2:num_samples
    q_prev = q_track(k - 1, :);
    T_target = eye(4);
    T_target(1:3, 1:3) = R_target;
    T_target(1:3, 4) = desired_positions(k, :).';
    q_track(k, :) = solve_inverse_step(robot, q_prev, T_target, desired_positions(k, :));
    condition_numbers(k) = condition_number(robot, q_track(k, :));
end
end

function q_next = solve_inverse_step(robot, q_seed, T_target, p_target)
q_next = reshape(q_seed, 1, []);
[warn_state, warn_msg_state, warn_id_state] = capture_warning_state();
cleanup_obj = onCleanup(@() restore_warning_state(warn_state, warn_msg_state, warn_id_state)); %#ok<NASGU>
warning('off', 'all');

try
    q_candidate = robot.ikine(T_target, q_next, [1 1 1 0 0 0]);
    if is_valid_joint_solution(robot, q_candidate)
        q_next = clamp_to_limits(robot, reshape(q_candidate, 1, []));
        return;
    end
catch
    % Fall back to damped least squares below.
end

q_next = solve_position_step_dls(robot, q_next, p_target);
end

function q_next = solve_position_step_dls(robot, q_seed, p_target)
q_next = reshape(q_seed, 1, []);
max_iterations = 50;
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

function tf = is_valid_joint_solution(robot, q_candidate)
tf = ~isempty(q_candidate) && isnumeric(q_candidate) && isvector(q_candidate) ...
    && numel(q_candidate) == robot.n && all(isfinite(q_candidate));
end

function q_clamped = clamp_to_limits(robot, q_row)
q_clamped = reshape(q_row, 1, []);
for idx = 1:robot.n
    q_clamped(idx) = min(max(q_clamped(idx), robot.links(idx).qlim(1)), robot.links(idx).qlim(2));
end
end

function cond_number = condition_number(robot, q_row)
J0 = robot.jacob0(q_row);
cond_number = cond(J0);
if ~isfinite(cond_number)
    cond_number = 1e12;
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

position_error = vecnorm(position_m - desired_positions, 2, 2);

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
result.desired_position_m = desired_positions;
end

function curve_length = path_length(points_xyz)
curve_length = sum(vecnorm(diff(points_xyz, 1, 1), 2, 2));
end

function rpy_deg = pose_from_transform(T)
R = T(1:3, 1:3);
yaw = atan2(R(2,1), R(1,1));
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
roll = atan2(R(3,2), R(3,3));
rpy_deg = rad2deg([roll, pitch, yaw]);
end

function plot_spatial_curve(desired_positions, actual_positions, title_text)
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
legend('Desired spatial curve', 'Actual path', 'Start', 'End', 'Location', 'best');
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

function plot_position_components(t, desired_positions, actual_positions, title_text)
figure('Name', title_text, 'Color', 'w');
axis_labels = {'X', 'Y', 'Z'};
for idx = 1:3
    subplot(3, 1, idx);
    plot(t, desired_positions(:, idx), 'b--', 'LineWidth', 1.4);
    hold on;
    plot(t, actual_positions(:, idx), 'r-', 'LineWidth', 1.3);
    grid on;
    ylabel(sprintf('%s (m)', axis_labels{idx}));
    if idx == 1
        title(title_text);
    end
    if idx == 3
        xlabel('Time (s)');
    end
end
legend('Desired', 'Actual', 'Location', 'best');
end

function plot_speed_profile(t, desired_positions, actual_positions, title_text)
dt = diff(t);
desired_speed = vecnorm(diff(desired_positions, 1, 1), 2, 2) ./ dt;
actual_speed = vecnorm(diff(actual_positions, 1, 1), 2, 2) ./ dt;
time_mid = (t(1:end-1) + t(2:end)) / 2;

figure('Name', title_text, 'Color', 'w');
plot(time_mid, desired_speed, 'b--', 'LineWidth', 1.5);
hold on;
plot(time_mid, actual_speed, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title(title_text);
legend('Desired speed', 'Actual speed', 'Location', 'best');
end

function plot_tracking_error(t, desired_positions, actual_positions, title_text)
err = vecnorm(actual_positions - desired_positions, 2, 2);
figure('Name', title_text, 'Color', 'w');
plot(t, err, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position error (m)');
title(title_text);
end

function plot_singularity_metric(t, condition_numbers, title_text)
figure('Name', title_text, 'Color', 'w');
semilogy(t, max(condition_numbers, 1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('cond(J)');
title(title_text);
end

function [warn_state, warn_msg_state, warn_id_state] = capture_warning_state()
warn_state = warning;
[warn_msg_state, warn_id_state] = lastwarn;
end

function restore_warning_state(warn_state, warn_msg_state, warn_id_state)
warning(warn_state);
if ~isempty(warn_msg_state) || ~isempty(warn_id_state)
    lastwarn(warn_msg_state, warn_id_state);
else
    lastwarn('', '');
end
end

function animate_robot_motion(robot, q_traj, tip_positions, figure_name, trail_color)
q_anim = q_traj(1:max(1, floor(size(q_traj, 1) / 80)):end, :);
tip_anim = tip_positions(1:max(1, floor(size(tip_positions, 1) / 80)):end, :);
workspace_bounds = compute_workspace_bounds([tip_positions; zeros(1, 3)]);

try
    figure_handle = figure('Name', figure_name, 'Color', 'w');
    robot.plot(q_anim(1, :), ...
        'workspace', workspace_bounds, ...
        'trail', {trail_color, 'LineWidth', 1.5}, ...
        'fps', 24, ...
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
radius = max(max(maxs - mins) / 2, 0.45);

workspace_bounds = [ ...
    center(1) - radius, center(1) + radius, ...
    center(2) - radius, center(2) + radius, ...
    center(3) - radius, center(3) + radius];
end
