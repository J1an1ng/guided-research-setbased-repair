%% Test meas_dist dimension

addpath(pwd);
addpath(fullfile(pwd, 'change_path'));

% Add CORA paths for Windows
cora_path = 'D:\MATLAB\CORABugfix+AROC\CORABugfix+AROC';
if isfolder(cora_path)
    addpath(fullfile(cora_path, 'cora'));
end

add_to_path_with_subfolders("models");
add_to_path_with_subfolders("primitive");
add_to_path_with_subfolders("utils");

% 測試當 meas_dist 为 zeros 时的行为
k = KSTInpDeltaStateOrder();

% 重现 generate_primitives_for_edgar 中的情况
state_meas_dist = zeros(k.get_size(), 1);
fprintf('state_meas_dist size: %d x %d\n', size(state_meas_dist, 1), size(state_meas_dist, 2));
fprintf('state_meas_dist is all zeros: %s\n', mat2str(state_meas_dist'));

% 创建 interval 和 zonotope
meas_dist_itv = interval(-state_meas_dist, state_meas_dist);
fprintf('interval dims: %d\n', meas_dist_itv.dim);

obj_meas_dist = zonotope(meas_dist_itv);
fprintf('zonotope dims: %d\n', obj_meas_dist.dim);

% 现在测试 StateProcessorForDiscrContrWithDistProc
dist_proc_temp = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc_temp);

fprintf('\nStateProcessor state size: %d\n', state_proc.get_state_size());
fprintf('Expected: 2 * %d + %d = %d\n', k.get_size(), obj_meas_dist.dim, 2*k.get_size() + obj_meas_dist.dim);
