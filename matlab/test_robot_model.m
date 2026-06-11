clear; clc; close all;

%% 1. 环境检查
fprintf('================ MATLAB Project Test ================\n');

if ~(exist('Link', 'class') == 8 && exist('SerialLink', 'class') == 8)
    error(['Robotics Toolbox for MATLAB not found. ', ...
        'Please install Peter Corke RTB and run startup_rvc first.']);
end

fprintf('Robotics Toolbox detected.\n');

%% 2. 定位配置文件
matlab_dir = fileparts(mfilename('fullpath'));
workspace_root = fileparts(matlab_dir);
config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');

if exist(config_path, 'file') ~= 2
    error('Config file not found: %s', config_path);
end

fprintf('Config file found: %s\n', config_path);

%% 3. 建立机器人模型
[robot, config, mdh_table] = build_robot_6dof(config_path);

fprintf('\nRobot model built successfully.\n');
fprintf('Robot name: %s\n', robot.name);
disp('MDH table:');
disp(mdh_table);

%% 4. 读取默认关节角
q_init_deg = reshape(double(config.default_state.q_init_deg), 1, []);
q_goal_deg = reshape(double(config.default_state.q_goal_deg), 1, []);
q_init = deg2rad(q_init_deg);
q_goal = deg2rad(q_goal_deg);

fprintf('\nDefault q_init (deg):\n');
disp(q_init_deg);
fprintf('Default q_goal (deg):\n');
disp(q_goal_deg);

%% 5. 正运动学测试
T_init = robot.fkine(q_init).T;
T_goal = robot.fkine(q_goal).T;

fprintf('\n================ FK Test ================\n');
fprintf('Initial pose T_init = \n');
disp(T_init);

fprintf('Goal pose T_goal = \n');
disp(T_goal);

p_init = T_init(1:3, 4).';
p_goal = T_goal(1:3, 4).';

fprintf('Initial end-effector position (m): [%.6f, %.6f, %.6f]\n', ...
    p_init(1), p_init(2), p_init(3));
fprintf('Goal end-effector position (m):    [%.6f, %.6f, %.6f]\n', ...
    p_goal(1), p_goal(2), p_goal(3));

%% 6. 雅可比矩阵测试
J0 = robot.jacob0(q_init);

fprintf('\n================ Jacobian Test ================\n');
fprintf('Size of J0: %d x %d\n', size(J0,1), size(J0,2));
disp(J0);

if all(size(J0) == [6, robot.n])
    fprintf('Jacobian dimension is correct.\n');
else
    error('Jacobian dimension is incorrect.');
end

condJ = cond(J0);
fprintf('cond(J0) = %.3e\n', condJ);

%% 7. 近邻逆运动学测试
fprintf('\n================ IK Test ================\n');

T_target = T_init;
T_target(1:3, 4) = T_target(1:3, 4) + [0.02; -0.01; 0.015];

ik_success = false;
q_ik = q_init;

try
    q_candidate = robot.ikcon(T_target, q_init);
    if ~isempty(q_candidate) && isnumeric(q_candidate) && numel(q_candidate) == robot.n ...
            && all(isfinite(q_candidate))
        q_ik = reshape(q_candidate, 1, []);
        ik_success = true;
        fprintf('IK solved by ikcon.\n');
    end
catch ME
    fprintf('ikcon failed: %s\n', ME.message);
end

if ~ik_success
    try
        q_candidate = robot.ikine(T_target, q_init, [1 1 1 0 0 0]);
        if ~isempty(q_candidate) && isnumeric(q_candidate) && numel(q_candidate) == robot.n ...
                && all(isfinite(q_candidate))
            q_ik = reshape(q_candidate, 1, []);
            ik_success = true;
            fprintf('IK solved by ikine(position-only).\n');
        end
    catch ME
        fprintf('ikine failed: %s\n', ME.message);
    end
end

if ik_success
    T_check = robot.fkine(q_ik).T;
    pos_err = norm(T_check(1:3,4) - T_target(1:3,4));
    fprintf('IK solution q (deg):\n');
    disp(rad2deg(q_ik));
    fprintf('Target position: [%.6f, %.6f, %.6f]\n', ...
        T_target(1), T_target(2), T_target(3)); %#ok<*NOPTS>
    fprintf('Reached position: [%.6f, %.6f, %.6f]\n', ...
        T_check(1,4), T_check(2,4), T_check(3,4));
    fprintf('Position error = %.6e m\n', pos_err);
else
    warning('IK test did not find a valid solution.');
end

%% 8. 简单绘图测试
fprintf('\n================ Plot Test ================\n');
figure('Name', 'Robot Initial Pose', 'Color', 'w');
robot.plot(q_init, 'workspace', [-0.8 0.8 -0.8 0.8 -0.4 0.8], 'scale', 0.7, 'jointdiam', 1.2);
title('Robot Initial Pose');
drawnow;
fprintf('Robot plot created successfully.\n');

%% 9. 调用空间曲线演示
fprintf('\n================ Spatial Curve Demo Test ================\n');
try
    run_spatial_curve_demo;
    fprintf('run_spatial_curve_demo finished successfully.\n');
catch ME
    fprintf('run_spatial_curve_demo failed.\n');
    fprintf('Error message:\n%s\n', ME.message);
end

fprintf('\n================ TEST FINISHED ================\n');