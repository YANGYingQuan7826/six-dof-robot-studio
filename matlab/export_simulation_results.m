function export_simulation_results(output_dir, result)
%EXPORT_SIMULATION_RESULTS Export MATLAB simulation results to CSV, JSON, MAT.

if nargin < 2
    error('Usage: export_simulation_results(output_dir, result)');
end

if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

joint_table = table(result.time_s(:), 'VariableNames', {'time_s'});
pose_table = table(result.time_s(:), 'VariableNames', {'time_s'});

num_joints = size(result.q_deg, 2);
for idx = 1:num_joints
    joint_table.(sprintf('q%d_deg', idx)) = result.q_deg(:, idx);
    joint_table.(sprintf('q%d_rad', idx)) = result.q_rad(:, idx);
end

pose_table.x_m = result.position_m(:, 1);
pose_table.y_m = result.position_m(:, 2);
pose_table.z_m = result.position_m(:, 3);
pose_table.roll_deg = result.rpy_deg(:, 1);
pose_table.pitch_deg = result.rpy_deg(:, 2);
pose_table.yaw_deg = result.rpy_deg(:, 3);

writetable(joint_table, fullfile(output_dir, 'joint_trajectory.csv'));
writetable(pose_table, fullfile(output_dir, 'pose_trajectory.csv'));
save(fullfile(output_dir, 'simulation_result.mat'), 'result');

json_result = result;
num_samples = size(result.transforms, 3);
transform_cells = cell(num_samples, 1);
for idx = 1:num_samples
    transform_cells{idx} = result.transforms(:, :, idx);
end
json_result.transforms = transform_cells;
json_text = jsonencode(json_result);

fid = fopen(fullfile(output_dir, 'simulation_result.json'), 'w');
if fid == -1
    error('Failed to open JSON output file for writing.');
end
cleanup_obj = onCleanup(@() fclose(fid));
fprintf(fid, '%s', json_text);
clear cleanup_obj;
end
