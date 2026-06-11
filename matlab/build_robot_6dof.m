function [robot, config, mdh_table] = build_robot_6dof(config_path)
%BUILD_ROBOT_6DOF Build a 6-DOF SerialLink robot from shared JSON config.
%   [ROBOT, CONFIG, MDH_TABLE] = BUILD_ROBOT_6DOF(CONFIG_PATH) reads the
%   shared Modified DH description and returns a Peter Corke SerialLink
%   model, the parsed config struct, and a MATLAB table for reporting.

if nargin < 1 || isempty(config_path)
    workspace_root = fileparts(fileparts(mfilename('fullpath')));
    config_path = fullfile(workspace_root, 'config', 'robot_mdh_template.json');
end

if ~(exist('Link', 'class') == 8 && exist('SerialLink', 'class') == 8)
    error(['Robotics Toolbox for MATLAB was not found on the path. ', ...
        'Please install Peter Corke RTB and run startup_rvc first.']);
end

if exist(config_path, 'file') ~= 2
    error('Config file not found: %s', config_path);
end

config = jsondecode(fileread(config_path));

if ~isfield(config, 'convention') || ~strcmpi(config.convention, 'modified_dh')
    error('This builder expects "modified_dh" in the shared config.');
end

num_links = numel(config.links);
link_cells = cell(1, num_links);

joint_labels = strings(num_links, 1);
joint_names = strings(num_links, 1);
a_vals = zeros(num_links, 1);
alpha_deg_vals = zeros(num_links, 1);
d_vals = zeros(num_links, 1);
offset_deg_vals = zeros(num_links, 1);
qmin_deg_vals = zeros(num_links, 1);
qmax_deg_vals = zeros(num_links, 1);

for idx = 1:num_links
    link_cfg = config.links(idx);

    theta = 0;
    d = link_cfg.d;
    a = link_cfg.a;
    alpha = deg2rad(link_cfg.alpha_deg);
    sigma = 0;
    offset = deg2rad(link_cfg.offset_deg);

    link = Link([theta d a alpha sigma offset], 'modified');
    qlim_deg = reshape(double(link_cfg.qlim_deg), 1, []);
    link.qlim = deg2rad(qlim_deg);
    link_cells{idx} = link;

    joint_labels(idx) = string(link_cfg.joint);
    joint_names(idx) = string(link_cfg.name);
    a_vals(idx) = a;
    alpha_deg_vals(idx) = link_cfg.alpha_deg;
    d_vals(idx) = d;
    offset_deg_vals(idx) = link_cfg.offset_deg;
    qmin_deg_vals(idx) = qlim_deg(1);
    qmax_deg_vals(idx) = qlim_deg(2);
end

robot = SerialLink([link_cells{:}], 'name', config.robot_name);
robot.base = make_transform(config.base_transform.translation_m, config.base_transform.rpy_deg);
robot.tool = make_transform(config.tool_transform.translation_m, config.tool_transform.rpy_deg);

mdh_table = table( ...
    (1:num_links).', ...
    joint_labels, ...
    joint_names, ...
    a_vals, ...
    alpha_deg_vals, ...
    d_vals, ...
    offset_deg_vals, ...
    qmin_deg_vals, ...
    qmax_deg_vals, ...
    'VariableNames', { ...
    'Index', 'Joint', 'Name', 'a_m', 'alpha_deg', 'd_m', 'offset_deg', 'Q0_deg', 'Q1_deg'});
end

function T = make_transform(translation_m, rpy_deg)
translation_m = reshape(double(translation_m), 1, 3);
rpy_rad = deg2rad(reshape(double(rpy_deg), 1, 3));

cr = cos(rpy_rad(1));
sr = sin(rpy_rad(1));
cp = cos(rpy_rad(2));
sp = sin(rpy_rad(2));
cy = cos(rpy_rad(3));
sy = sin(rpy_rad(3));

Rx = [1 0 0; 0 cr -sr; 0 sr cr];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rz = [cy -sy 0; sy cy 0; 0 0 1];

R = Rz * Ry * Rx;
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = translation_m(:);
end
