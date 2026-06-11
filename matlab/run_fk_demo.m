clear; clc;

workspace_root = fileparts(fileparts(mfilename('fullpath')));
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');

[robot, config, mdh_table] = build_robot_6dof(config_path);

q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_goal_deg = reshape(double(config.default_state.q_goal_deg), 1, []);

fprintf('================ 6-DOF MDH Table ================\n');
disp(mdh_table);

fprintf('\n================ FK Demo ================\n');
fprintf('Robot name: %s\n', config.robot_name);

run_fk_case(robot, 'Initial pose', q_init_deg);
run_fk_case(robot, 'Goal pose', q_goal_deg);

disp('FK demo finished.');

function run_fk_case(robot, title_text, q_deg)
q_rad = deg2rad(q_deg);
T = robot.fkine(q_rad).T;
[position_m, rpy_deg] = pose_from_transform(T);

fprintf('\n%s\n', title_text);
fprintf('q (deg) = [%s]\n', join(string(round(q_deg, 3)), ', '));
disp('T = ');
disp(T);
fprintf('Position (m) = [%.4f, %.4f, %.4f]\n', position_m(1), position_m(2), position_m(3));
fprintf('RPY (deg, ZYX intrinsic equivalent) = [%.3f, %.3f, %.3f]\n', ...
    rpy_deg(1), rpy_deg(2), rpy_deg(3));
end

function [position_m, rpy_deg] = pose_from_transform(T)
position_m = T(1:3, 4).';
R = T(1:3, 1:3);

yaw = atan2(R(2,1), R(1,1));
pitch = atan2(-R(3,1), hypot(R(3,2), R(3,3)));
roll = atan2(R(3,2), R(3,3));
rpy_deg = rad2deg([roll, pitch, yaw]);
end
