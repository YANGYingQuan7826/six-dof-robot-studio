clear; clc; close all;

workspace_root = fileparts(fileparts(mfilename('fullpath')));
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');
output_root = fullfile(workspace_root, 'outputs', 'matlab_endpoint7_circle_demo');

[robot, config, mdh_table] = build_robot_6dof(config_path);

q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_init = deg2rad(q_init_deg);
duration_s = double(config.simulation_defaults.duration_s);
steps = double(config.simulation_defaults.steps);
t = linspace(0, duration_s, steps).';

disp('================ Endpoint-7 Circle Demo MDH Table ================');
disp(mdh_table);

[q_traj, desired_p6, desired_p7, info] = make_endpoint7_circle_track(robot, config, q_init, t);
result = evaluate_endpoint7_trajectory( ...
    robot, q_traj, t, desired_p6, desired_p7, info.endpoint7_offset_local, ...
    info.position_condition_numbers, info.orientation_condition_numbers, info.singularity_count);
export_simulation_results(output_root, result.base_result);

fprintf('\nEndpoint-7 circle demo summary:\n');
fprintf('Assessment: %s\n', info.assessment);
fprintf('Requested radius = %.4f m\n', info.requested_radius_m);
fprintf('Used radius      = %.4f m\n', info.used_radius_m);
fprintf('Plane normal     = [%.3f, %.3f, %.3f]\n', info.plane_normal(1), info.plane_normal(2), info.plane_normal(3));
fprintf('Endpoint-7 max error = %.6f m\n', result.endpoint7_metrics.max_position_error_m);
fprintf('Endpoint-7 mean error = %.6f m\n', result.endpoint7_metrics.mean_position_error_m);
fprintf('Reachable ratio (<=5 mm) = %.2f %%\n', 100 * result.endpoint7_metrics.reachable_ratio);
fprintf('Near-singular samples = %d\n', result.singularity_count);
fprintf('Output folder: %s\n', output_root);

plot_endpoint7_circle(desired_p7, result.endpoint7_position_m, 'Endpoint-7 spatial circle');
plot_endpoint6_path(desired_p6, result.endpoint6_position_m, 'Endpoint-6 compensated path');
plot_joint_trajectory(t, result.base_result.q_deg, 'Endpoint-7 circle joint motion');
plot_endpoint7_error(t, result.endpoint7_error_norm, 'Endpoint-7 tracking error');
plot_singularity_curves(t, result.position_condition_numbers, result.orientation_condition_numbers, ...
    'Endpoint-7 circle singularity metrics');
animate_robot_motion(robot, q_traj, result.endpoint7_position_m, 'Endpoint-7 circle robot motion', [0.2 0.55 0.85], info.endpoint7_offset_local);

function [q_traj, desired_p6, desired_p7, info] = make_endpoint7_circle_track(robot, config, q_start, t)
num_samples = numel(t);
T6_start = robot.fkine(q_start).T;
R_target = T6_start(1:3, 1:3);
endpoint7_offset_local = reshape(double(config.endpoint7.offset_in_frame6_m), 3, 1);
p6_start = T6_start(1:3, 4).';
p7_start = p6_start + (R_target * endpoint7_offset_local).';

requested_radius = double(config.simulation_defaults.endpoint7_circle_radius_m);
center_hint = reshape(double(config.simulation_defaults.endpoint7_circle_center_offset_m), 1, 3);
plane_normal = reshape(double(config.simulation_defaults.endpoint7_circle_plane_normal), 1, 3);

radius_candidates = [requested_radius, 0.8 * requested_radius, 0.6 * requested_radius, 0.4 * requested_radius];
best = struct('max_err', inf, 'q_traj', [], 'desired_p6', [], 'desired_p7', [], 'used_radius', requested_radius, ...
    'position_cond', [], 'orientation_cond', [], 'singularity_count', 0);

phi_ref = sum(q_start(1:3));
for idx = 1:numel(radius_candidates)
    radius = max(radius_candidates(idx), 0.005);
    [center, basis_u, basis_v, plane_normal_unit] = build_spatial_circle_frame(p7_start, center_hint, plane_normal, radius);

    desired_p7_candidate = zeros(num_samples, 3);
    for k = 1:num_samples
        phase = 2 * pi * (k - 1) / max(num_samples - 1, 1);
        desired_p7_candidate(k, :) = center + radius * (basis_u * cos(phase) + basis_v * sin(phase));
    end

    desired_p6_candidate = desired_p7_candidate - repmat((R_target * endpoint7_offset_local).', num_samples, 1);
    [q_candidate, position_cond, orientation_cond, singularity_count] = solve_endpoint7_track( ...
        robot, q_start, desired_p6_candidate, R_target, phi_ref);

    actual_p7_candidate = endpoint7_positions_from_q(robot, q_candidate, endpoint7_offset_local);
    endpoint7_err = vecnorm(actual_p7_candidate - desired_p7_candidate, 2, 2);
    max_err = max(endpoint7_err);

    if max_err < best.max_err
        best.max_err = max_err;
        best.q_traj = q_candidate;
        best.desired_p6 = desired_p6_candidate;
        best.desired_p7 = desired_p7_candidate;
        best.used_radius = radius;
        best.position_cond = position_cond;
        best.orientation_cond = orientation_cond;
        best.singularity_count = singularity_count;
    end

    if max_err < 5e-3
        break;
    end
end

assessment = "approximate";
if best.max_err < 5e-3
    assessment = "can_draw_stable_spatial_circle";
elseif best.max_err < 15e-3
    assessment = "can_draw_approximate_spatial_circle";
end

q_traj = best.q_traj;
desired_p6 = best.desired_p6;
desired_p7 = best.desired_p7;
info = struct( ...
    'assessment', char(assessment), ...
    'requested_radius_m', requested_radius, ...
    'used_radius_m', best.used_radius, ...
    'plane_normal', plane_normal_unit, ...
    'position_condition_numbers', best.position_cond, ...
    'orientation_condition_numbers', best.orientation_cond, ...
    'singularity_count', best.singularity_count, ...
    'endpoint7_offset_local', endpoint7_offset_local);
end

function [q_traj, position_cond, orientation_cond, singularity_count] = solve_endpoint7_track(robot, q_start, desired_p6, R_target, phi_ref)
num_samples = size(desired_p6, 1);
q_traj = zeros(num_samples, robot.n);
q_traj(1, :) = q_start;
position_cond = zeros(num_samples, 1);
orientation_cond = zeros(num_samples, 1);
singularity_count = 0;

for k = 2:num_samples
    [q_traj(k, :), near_singular, cond_pos, cond_ori] = solve_decomposed_ik_step( ...
        robot, q_traj(k - 1, :), desired_p6(k, :), R_target, phi_ref);
    position_cond(k) = cond_pos;
    orientation_cond(k) = cond_ori;
    singularity_count = singularity_count + double(near_singular);
end
end

function [q_next, near_singular, cond_pos, cond_ori] = solve_decomposed_ik_step(robot, q_seed, p6_target, R_target, phi_ref)
q_next = reshape(q_seed, 1, []);
q_next = solve_first_three_closed_form(robot, q_next, p6_target, phi_ref);
q_next = solve_last_three_closed_form(robot, q_next, R_target);
q_next = clamp_to_limits(robot, q_next);

T_target = eye(4);
T_target(1:3, 1:3) = R_target;
T_target(1:3, 4) = p6_target(:);

[warn_state, warn_msg_state, warn_id_state] = capture_warning_state();
cleanup_obj = onCleanup(@() restore_warning_state(warn_state, warn_msg_state, warn_id_state));
warning('off', 'all');
try
    q_refined = robot.ikcon(T_target, q_next);
    if is_valid_joint_solution(robot, q_refined)
        q_next = clamp_to_limits(robot, reshape(q_refined, 1, []));
    end
catch
end

J0 = robot.jacob0(q_next);
cond_pos = cond(J0(1:3, 1:3));
cond_ori = cond(J0(4:6, 4:6));
near_singular = (isfinite(cond_pos) && cond_pos > 1e4) || (isfinite(cond_ori) && cond_ori > 1e4);
end

function q_row = solve_first_three_closed_form(robot, q_seed, p6_target, phi_ref)
q_row = reshape(q_seed, 1, []);
L1 = robot.links(2).a;
L2 = robot.links(3).a;
d4 = robot.links(4).d;

px = p6_target(1) - d4 * sin(phi_ref);
pz = p6_target(3) + d4 * cos(phi_ref);
r2 = px^2 + pz^2;
c2 = (r2 - L1^2 - L2^2) / (2 * L1 * L2);
c2 = min(max(c2, -1), 1);
s2_candidates = [sqrt(max(0, 1 - c2^2)), -sqrt(max(0, 1 - c2^2))];

best_q123 = q_row(1:3);
best_cost = inf;
for idx = 1:numel(s2_candidates)
    s2 = s2_candidates(idx);
    q2 = atan2(s2, c2);
    q1 = atan2(pz, px) - atan2(L2 * s2, L1 + L2 * c2);
    q3 = phi_ref - q1 - q2;
    q123 = [q1, q2, q3];
    q_candidate = clamp_to_limits(robot, [q123, q_row(4:6)]);
    cost = norm(wrap_to_pi(q_candidate(1:3) - q_row(1:3)));
    if cost < best_cost
        best_cost = cost;
        best_q123 = q_candidate(1:3);
    end
end
q_row(1:3) = best_q123;
end

function q_row = solve_last_three_closed_form(robot, q_row, R_target)
R03 = rotation03_from_q123(q_row(1:3));
R36 = R03' * R_target;

q4 = atan2(-R36(1, 3), R36(3, 3));
q56 = atan2(R36(2, 1), R36(2, 2));
q5 = q_row(5);
q6 = wrap_to_pi_scalar(q56 - q5);
q_row = clamp_to_limits(robot, [q_row(1:3), q4, q5, q6]);
end

function R03 = rotation03_from_q123(q123)
phi = q123(1) + q123(2) + q123(3);
R03 = [cos(phi), -sin(phi), 0; ...
       0,         0,        -1; ...
       sin(phi),  cos(phi),  0];
end

function [center, basis_u, basis_v, plane_normal_unit] = build_spatial_circle_frame(start_point, center_hint, plane_normal, radius)
plane_normal_unit = reshape(plane_normal, 1, 3);
if norm(plane_normal_unit) < 1e-9
    plane_normal_unit = [1, 1, 1];
end
plane_normal_unit = plane_normal_unit / norm(plane_normal_unit);

hint = center_hint - dot(center_hint, plane_normal_unit) * plane_normal_unit;
if norm(hint) < 1e-9
    arbitrary = [1, 0, 0];
    if abs(dot(arbitrary, plane_normal_unit)) > 0.9
        arbitrary = [0, 1, 0];
    end
    hint = arbitrary - dot(arbitrary, plane_normal_unit) * plane_normal_unit;
end

basis_u = hint / norm(hint);
basis_v = cross(plane_normal_unit, basis_u);
basis_v = basis_v / norm(basis_v);
center = start_point - radius * basis_u;
end

function positions = endpoint7_positions_from_q(robot, q_traj, offset_local)
positions = zeros(size(q_traj, 1), 3);
for k = 1:size(q_traj, 1)
    T6 = robot.fkine(q_traj(k, :)).T;
    positions(k, :) = T6(1:3, 4).' + (T6(1:3, 1:3) * offset_local).';
end
end

function result = evaluate_endpoint7_trajectory(robot, q_traj, t, desired_p6, desired_p7, endpoint7_offset_local, ...
    position_condition_numbers, orientation_condition_numbers, singularity_count)
num_samples = size(q_traj, 1);
endpoint6_position_m = zeros(num_samples, 3);
endpoint7_position_m = zeros(num_samples, 3);
rpy_deg = zeros(num_samples, 3);
transforms = zeros(4, 4, num_samples);

for k = 1:num_samples
    T6 = robot.fkine(q_traj(k, :)).T;
    transforms(:, :, k) = T6;
    endpoint6_position_m(k, :) = T6(1:3, 4).';
    endpoint7_position_m(k, :) = T6(1:3, 4).' + (T6(1:3, 1:3) * endpoint7_offset_local).';
    rpy_deg(k, :) = pose_from_transform(T6);
end

endpoint6_error = vecnorm(endpoint6_position_m - desired_p6, 2, 2);
endpoint7_error = vecnorm(endpoint7_position_m - desired_p7, 2, 2);

base_result = struct();
base_result.summary = struct( ...
    'robot_name', robot.name, ...
    'source', 'matlab', ...
    'trajectory_type', 'endpoint7_spatial_circle', ...
    'time_span_s', t(end) - t(1), ...
    'samples', num_samples);
base_result.time_s = t;
base_result.q_rad = q_traj;
base_result.q_deg = rad2deg(q_traj);
base_result.position_m = endpoint6_position_m;
base_result.rpy_deg = rpy_deg;
base_result.transforms = transforms;
base_result.metrics = struct( ...
    'max_position_error_m', max(endpoint6_error), ...
    'mean_position_error_m', mean(endpoint6_error));
base_result.desired_position_m = desired_p6;

result = struct( ...
    'base_result', base_result, ...
    'endpoint6_position_m', endpoint6_position_m, ...
    'endpoint7_position_m', endpoint7_position_m, ...
    'endpoint7_error_norm', endpoint7_error, ...
    'endpoint7_metrics', struct( ...
        'max_position_error_m', max(endpoint7_error), ...
        'mean_position_error_m', mean(endpoint7_error), ...
        'reachable_ratio', mean(endpoint7_error <= 5e-3)), ...
    'position_condition_numbers', position_condition_numbers, ...
    'orientation_condition_numbers', orientation_condition_numbers, ...
    'singularity_count', singularity_count);
end

function rpy_deg = pose_from_transform(T)
R = T(1:3, 1:3);
yaw = atan2(R(2,1), R(1,1));
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
roll = atan2(R(3,2), R(3,3));
rpy_deg = rad2deg([roll, pitch, yaw]);
end

function tf = is_valid_joint_solution(robot, q_candidate)
tf = ~isempty(q_candidate) && isnumeric(q_candidate) && isvector(q_candidate) ...
    && numel(q_candidate) == robot.n && all(isfinite(q_candidate));
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

function q_clamped = clamp_to_limits(robot, q_row)
q_clamped = reshape(q_row, 1, []);
for idx = 1:robot.n
    q_clamped(idx) = min(max(q_clamped(idx), robot.links(idx).qlim(1)), robot.links(idx).qlim(2));
end
end

function angles_wrapped = wrap_to_pi(angles)
angles_wrapped = arrayfun(@wrap_to_pi_scalar, angles);
end

function angle_wrapped = wrap_to_pi_scalar(angle_value)
angle_wrapped = atan2(sin(angle_value), cos(angle_value));
end

function plot_endpoint7_circle(desired_positions, actual_positions, title_text)
figure('Name', title_text, 'Color', 'w');
plot3(desired_positions(:, 1), desired_positions(:, 2), desired_positions(:, 3), 'b--', 'LineWidth', 1.6);
hold on;
plot3(actual_positions(:, 1), actual_positions(:, 2), actual_positions(:, 3), 'r-', 'LineWidth', 1.6);
plot3(actual_positions(1, 1), actual_positions(1, 2), actual_positions(1, 3), 'go', 'MarkerFaceColor', 'g');
plot3(actual_positions(end, 1), actual_positions(end, 2), actual_positions(end, 3), 'ko', 'MarkerFaceColor', 'k');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(title_text);
legend('Desired circle', 'Actual endpoint-7 path', 'Start', 'End', 'Location', 'best');
end

function plot_endpoint6_path(desired_positions, actual_positions, title_text)
figure('Name', title_text, 'Color', 'w');
plot3(desired_positions(:, 1), desired_positions(:, 2), desired_positions(:, 3), 'b--', 'LineWidth', 1.4);
hold on;
plot3(actual_positions(:, 1), actual_positions(:, 2), actual_positions(:, 3), 'r-', 'LineWidth', 1.4);
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(title_text);
legend('Desired endpoint-6 path', 'Actual endpoint-6 path', 'Location', 'best');
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

function plot_endpoint7_error(t, err_norm, title_text)
figure('Name', title_text, 'Color', 'w');
plot(t, err_norm, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Endpoint-7 error (m)');
title(title_text);
end

function plot_singularity_curves(t, cond_pos, cond_ori, title_text)
figure('Name', title_text, 'Color', 'w');
subplot(2,1,1);
semilogy(t, max(cond_pos, 1), 'LineWidth', 1.3);
grid on;
ylabel('cond(Jp)');
title(title_text);

subplot(2,1,2);
semilogy(t, max(cond_ori, 1), 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('cond(Jw)');
end

function animate_robot_motion(robot, q_traj, tip_positions, figure_name, trail_color, endpoint7_offset_local)
q_anim = q_traj(1:max(1, floor(size(q_traj, 1) / 80)):end, :);
tip_anim = tip_positions(1:max(1, floor(size(tip_positions, 1) / 80)):end, :);
workspace_bounds = compute_workspace_bounds(tip_positions);

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
    animate_robot_motion_fallback(robot, q_anim, tip_anim, figure_name, trail_color, workspace_bounds, endpoint7_offset_local);
end
end

function animate_robot_motion_fallback(robot, q_traj, tip_positions, figure_name, trail_color, workspace_bounds, endpoint7_offset_local)
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
endpoint7_trace = animatedline('Color', trail_color, 'LineWidth', 1.8);
endpoint7_pt = plot3(0, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

for k = 1:size(q_traj, 1)
    joint_positions = compute_joint_positions(robot, q_traj(k, :), endpoint7_offset_local);
    set(robot_line, ...
        'XData', joint_positions(:, 1), ...
        'YData', joint_positions(:, 2), ...
        'ZData', joint_positions(:, 3));
    set(endpoint7_pt, 'XData', joint_positions(end,1), 'YData', joint_positions(end,2), 'ZData', joint_positions(end,3));
    addpoints(endpoint7_trace, joint_positions(end, 1), joint_positions(end, 2), joint_positions(end, 3));
    drawnow;
    pause(0.03);
end
end

function joint_positions = compute_joint_positions(robot, q_row, endpoint7_offset_local)
q_row = reshape(q_row, 1, []);
joint_positions = zeros(robot.n + 3, 3);

T = transform_to_matrix(robot.base);
joint_positions(1, :) = T(1:3, 4).';

for idx = 1:robot.n
    A_idx = robot.links(idx).A(q_row(idx));
    T = T * transform_to_matrix(A_idx);
    joint_positions(idx + 1, :) = T(1:3, 4).';
end

tool_transform = T * transform_to_matrix(robot.tool);
joint_positions(robot.n + 2, :) = tool_transform(1:3, 4).';
endpoint7_pos = tool_transform(1:3, 4) + tool_transform(1:3, 1:3) * endpoint7_offset_local;
joint_positions(robot.n + 3, :) = endpoint7_pos.';
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
