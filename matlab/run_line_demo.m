clear; clc; close all;

workspace_root = fileparts(fileparts(mfilename('fullpath')));
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');
output_root = fullfile(workspace_root, 'outputs', 'matlab_line_demo');

[robot, config, mdh_table] = build_robot_6dof(config_path);

q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_init = deg2rad(q_init_deg);
duration_s = double(config.simulation_defaults.duration_s);
steps = double(config.simulation_defaults.steps);
t = linspace(0, duration_s, steps).';

disp('================ Line Demo MDH Table ================');
disp(mdh_table);

[q_line, desired_positions, line_info] = make_line_track(robot, config, q_init, t);
line_result = evaluate_trajectory(robot, q_line, t, desired_positions, 'matlab', 'line_track');
export_simulation_results(output_root, line_result);

fprintf('\nLine demo summary:\n');
fprintf('IK method: %s\n', line_info.ik_method);
fprintf('Line offset (m) = [%.4f, %.4f, %.4f]\n', line_info.line_offset_m(1), line_info.line_offset_m(2), line_info.line_offset_m(3));
fprintf('Max position error = %.6f m\n', line_result.metrics.max_position_error_m);
fprintf('Mean position error = %.6f m\n', line_result.metrics.mean_position_error_m);
fprintf('Max orientation error = %.6f deg\n', line_info.max_orientation_error_deg);
fprintf('Near-singular samples = %d\n', line_info.singularity_count);
fprintf('Max wrist-center error = %.6f m\n', line_info.max_wrist_center_error_m);

plot_line_track(desired_positions, line_result.position_m, 'End-effector straight-line motion');
plot_joint_trajectory(t, line_result.q_deg, 'Straight-line joint motion');
plot_speed_profile(t, desired_positions, line_result.position_m, 'Straight-line speed profile');
plot_orientation_hold(t, line_info.orientation_errors_deg, 'Orientation hold error');
plot_singularity_curves(t, line_info.position_condition_numbers, line_info.orientation_condition_numbers, ...
    'Singularity metrics');
animate_robot_motion(robot, q_line, line_result.position_m, 'Straight-line robot motion', [0.85 0.35 0.15]);

fprintf('Line demo output folder: %s\n', output_root);

function [q_track, desired_positions, info] = make_line_track(robot, config, q_start, t)
num_samples = numel(t);
T_start = robot.fkine(q_start).T;
start_position = T_start(1:3, 4).';
target_rotation = T_start(1:3, 1:3);
line_offset = reshape(double(config.simulation_defaults.line_track_offset_m), 1, 3);
end_position = start_position + line_offset;

desired_positions = zeros(num_samples, 3);
for k = 1:num_samples
    tau = (k - 1) / max(num_samples - 1, 1);
    desired_positions(k, :) = start_position + tau * (end_position - start_position);
end

q_track = zeros(num_samples, robot.n);
q_track(1, :) = q_start;
orientation_errors_deg = zeros(num_samples, 1);
singularity_count = 0;
position_condition_numbers = zeros(num_samples, 1);
orientation_condition_numbers = zeros(num_samples, 1);
wrist_center_errors = zeros(num_samples, 1);
phi_ref = sum(q_start(1:3));

for k = 2:num_samples
    [q_track(k, :), near_singular, orientation_errors_deg(k), position_condition_numbers(k), ...
        orientation_condition_numbers(k), wrist_center_errors(k)] = solve_decomposed_ik_step( ...
        robot, q_track(k - 1, :), desired_positions(k, :), target_rotation, phi_ref);
    singularity_count = singularity_count + double(near_singular);
end

info = struct( ...
    'ik_method', 'closed_form_wrist_center_plus_r36_refinement', ...
    'line_offset_m', line_offset, ...
    'max_orientation_error_deg', max(orientation_errors_deg), ...
    'orientation_errors_deg', orientation_errors_deg, ...
    'singularity_count', singularity_count, ...
    'position_condition_numbers', position_condition_numbers, ...
    'orientation_condition_numbers', orientation_condition_numbers, ...
    'max_wrist_center_error_m', max(wrist_center_errors));
end

function [q_next, near_singular, orientation_error_deg, cond_pos, cond_ori, wrist_center_error] = ...
    solve_decomposed_ik_step(robot, q_seed, p_target, R_target, phi_ref)
q_next = reshape(q_seed, 1, []);
target_wrist_center = wrist_center_from_pose(robot, p_target, R_target);
q_next = solve_first_three_closed_form(robot, q_next, target_wrist_center, phi_ref);
q_next = solve_last_three_closed_form(robot, q_next, R_target);
q_next = clamp_to_limits(robot, q_next);

T_target = eye(4);
T_target(1:3, 1:3) = R_target;
T_target(1:3, 4) = p_target(:);

[warn_state, warn_msg_state, warn_id_state] = capture_warning_state();
cleanup_obj = onCleanup(@() restore_warning_state(warn_state, warn_msg_state, warn_id_state));
warning('off', 'all');
try
    q_refined = robot.ikcon(T_target, q_next);
    if is_valid_joint_solution(robot, q_refined)
        q_next = clamp_to_limits(robot, reshape(q_refined, 1, []));
    end
catch
    % Keep the decomposed solution if RTB refinement fails.
end

J0 = robot.jacob0(q_next);
cond_pos = cond(J0(1:3, 1:3));
cond_ori = cond(J0(4:6, 4:6));
near_singular = (isfinite(cond_pos) && cond_pos > 1e4) || (isfinite(cond_ori) && cond_ori > 1e4);

T_final = robot.fkine(q_next).T;
orientation_error_deg = rad2deg(norm(rotation_error_vector(T_final(1:3, 1:3), R_target)));
wrist_center_error = norm(target_wrist_center - wrist_center_from_q(robot, q_next));
end

function q_row = solve_first_three_closed_form(robot, q_seed, target_wrist_center, phi_ref)
q_row = reshape(q_seed, 1, []);
L1 = robot.links(2).a;
L2 = robot.links(3).a;
d4 = robot.links(4).d;

px = target_wrist_center(1) - d4 * sin(phi_ref);
pz = target_wrist_center(3) + d4 * cos(phi_ref);
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
q_candidate = [q_row(1:3), q4, q5, q6];
q_row = clamp_to_limits(robot, q_candidate);
end

function R03 = rotation03_from_q123(q123)
phi = q123(1) + q123(2) + q123(3);
R03 = [cos(phi), -sin(phi), 0; ...
       0,         0,        -1; ...
       sin(phi),  cos(phi),  0];
end

function err_vec = rotation_error_vector(R_current, R_target)
err_vec = 0.5 * ( ...
    cross(R_current(:, 1), R_target(:, 1)) + ...
    cross(R_current(:, 2), R_target(:, 2)) + ...
    cross(R_current(:, 3), R_target(:, 3)));
end

function p_wc = wrist_center_from_pose(robot, p_target, R_target)
offset_local = wrist_offset_local(robot);
p_wc = reshape(p_target, 1, 3) - (R_target * offset_local).';
end

function p_wc = wrist_center_from_q(robot, q_row)
T = robot.fkine(q_row).T;
offset_local = wrist_offset_local(robot);
p_wc = T(1:3, 4).' - (T(1:3, 1:3) * offset_local).';
end

function offset_local = wrist_offset_local(robot)
tool_T = transform_to_matrix(robot.tool);
tool_translation = tool_T(1:3, 4);
offset_local = [0; 0; robot.links(6).d] + tool_translation;
end

function angles_wrapped = wrap_to_pi(angles)
angles_wrapped = arrayfun(@wrap_to_pi_scalar, angles);
end

function angle_wrapped = wrap_to_pi_scalar(angle_value)
angle_wrapped = atan2(sin(angle_value), cos(angle_value));
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

function plot_line_track(desired_positions, actual_positions, title_text)
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
legend('Desired line', 'Actual path', 'Start', 'End', 'Location', 'best');
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

function plot_orientation_hold(t, orientation_errors_deg, title_text)
figure('Name', title_text, 'Color', 'w');
plot(t, orientation_errors_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Orientation error (deg)');
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

function animate_robot_motion(robot, q_traj, tip_positions, figure_name, trail_color)
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
