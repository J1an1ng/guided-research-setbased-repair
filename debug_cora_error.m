%% Manually create the system and attempt reach to find exact CORA error

% Setup paths
addpath(pwd);
addpath(fullfile(pwd, 'change_path'));

% For Windows: Use CORABugfix+AROC if available
cora_path = 'D:\MATLAB\CORABugfix+AROC\CORABugfix+AROC';
if isfolder(cora_path)
    addpath(fullfile(cora_path, 'cora'));
    addpath(fullfile(cora_path, 'AROC'));
else
    % Fallback: try to use available toolboxes
    try
        add_latest_cora();
    catch
        fprintf('Warning: Could not add CORA paths\n');
    end
end

add_to_path_with_subfolders("models");
add_to_path_with_subfolders("controllers");
add_to_path_with_subfolders("controlled_models");
add_to_path_with_subfolders("generation");
add_to_path_with_subfolders("configs");
add_to_path_with_subfolders("cora_options");
add_to_path_with_subfolders("utils");
add_to_path_with_subfolders("primitive");

fid = fopen('debug_cora_error.log', 'w');
fprintf(fid, 'CORA Debug Log\n\n');

try
    %% Setup (mimic generate_primitives_for_edgar)
    k = KSTInpDeltaStateOrder();
    i = InpOrderADelta();
    params = create_kst_params_edgar();
    
    % Create state processor
    state_meas_dist = zeros(k.get_size(), 1);
    dist_proc = KSTDistProcForRotatedPositionalDists(k);
    state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);
    
    model_add_dist = zeros(k.get_size(), 1);
    
    % Create dynamics
    ref_state_builder = SameRefStateBuilder(k);
    control_params = [18.5, 72.8, 18, 19.7692, 8.3620];
    
    dyn_conf = ControlledModelParamConfig(false, ...
        k, i, ...
        false, true, false, ...
        ref_state_builder.get_ref_state_size(), true, length(control_params));
    
    dyn_conf_values = ControlledModelParamValues( ...
        [], model_add_dist, [], [], ...
        ref_state_builder, control_params);
    
    [system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
        @kst_inp_delta, @pd_controller_inp_delta_4d, ...
        k, i, params, dyn_conf, state_proc);
    
    fprintf(fid, '=== System Information ===\n');
    fprintf(fid, 'State processor state size: %d\n', state_proc.get_state_size());
    fprintf(fid, 'State order size: %d\n', k.get_size());
    fprintf(fid, 'Param config size: %d\n', dyn_conf.get_params_size());
    
    % Create nonlinearSys
    fprintf(fid, '\nCreating nonlinear system...\n');
    sys = nonlinearSys('edgar_closed_loop_system_inp_delta_4d_test', ...
        system_dynamics, ...
        state_proc.get_state_size(), dyn_conf.get_params_size());
    
    fprintf(fid, 'System created successfully\n');
    fprintf(fid, 'System state dimension: %d\n', length(sys));
    fprintf(fid, 'System input dimension: %d\n', sys.nrOfInputs);
    
    % Try to create initial set
    fprintf(fid, '\nCreating initial set...\n');
    x0_interval = interval([-0.09, -0.03, -0.09, -0.0005, -0.09, -0.03, -0.09, -0.0005], ...
                           [0.09, 0.03, 0.09, 0.0005, 0.09, 0.03, 0.09, 0.0005]);
    R0 = zonotope(x0_interval);
    fprintf(fid, 'Initial set created, dimension: %d\n', dimension(R0));
    
    % Try reach with minimal setup
    fprintf(fid, '\nSetting up CORA options...\n');
    cora_options = create_cora_options_1();
    cora_options.timeStep = 0.01;
    cora_options.maxError = Inf * ones(state_proc.get_state_size(), 1);
    
    fprintf(fid, 'CORA options set. Starting reach...\n');
    fclose(fid);  % Flush before reach
    
    Rall = reach(sys, R0, dyn_conf_values, cora_options);
    
    fid = fopen('debug_cora_error.log', 'a');
    fprintf(fid, 'Reach completed successfully\n');
    
catch ME
    fid = fopen('debug_cora_error.log', 'a');
    fprintf(fid, '\n=== ERROR CAUGHT ===\n');
    fprintf(fid, 'Message: %s\n', ME.message);
    fprintf(fid, '\nStack:\n');
    for i = 1:length(ME.stack)
        fprintf(fid, '  %s (line %d) in %s\n', ME.stack(i).file, ME.stack(i).line, ME.stack(i).name);
    end
end

if ~isempty(fid) && fid > 0
    fclose(fid);
end

% Display result
if exist('debug_cora_error.log', 'file')
    type debug_cora_error.log
end
