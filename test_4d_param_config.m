%% Add paths and test 4D config

% Add paths manually using the project's standard method
addpath(pwd);  % for add_to_path_with_subfolders
addpath(fullfile(pwd, 'change_path'));  % for add_to_path_with_subfolders
add_to_path_with_subfolders("models");
add_to_path_with_subfolders("controllers");
add_to_path_with_subfolders("controlled_models");

% Quick test to validate 4D parameter configuration
k = KSTInpDeltaStateOrder();
i = InpOrderADelta();

% Check state/input size
fprintf('=== State and Input Order ===\n');
fprintf('State size: %d\n', k.get_size());
fprintf('State indices: [X=%d, Y=%d, V=%d, PSI=%d]\n', k.X, k.Y, k.V, k.PSI);
fprintf('Input size: %d\n', i.SIZE);

% Check parameter config size
ref_state_size = 4; % SameRefStateBuilder(k)
control_weights_size = 5;
include_ref_inp = true;
include_model_add_dist = true;
include_meas_dist = false;
include_model_inp_dist = false;

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    include_meas_dist, include_model_add_dist, include_model_inp_dist, ...
    ref_state_size, include_ref_inp, control_weights_size);

fprintf('\n=== ControlledModelParamConfig ===\n');
fprintf('Total params size: %d\n', dyn_conf.get_params_size());
fprintf('Expected: 4 (model_add_dist) + 4 (ref_state) + 2 (ref_inp) + 5 (control_weights) = 15\n');

% Break down
fprintf('\nBreakdown:\n');
fprintf('  include_model_add_dist = %d, state_size = %d -> %d\n', include_model_add_dist, k.get_size(), double(include_model_add_dist) * k.get_size());
fprintf('  ref_state_size = %d\n', ref_state_size);
fprintf('  include_ref_inp = %d, inp_size = %d -> %d\n', include_ref_inp, i.SIZE, double(include_ref_inp) * i.SIZE);
fprintf('  control_weights_size = %d\n', control_weights_size);
fprintf('\nTotal = %d + %d + %d + %d = %d\n', ...
    double(include_model_add_dist) * k.get_size(), ...
    ref_state_size, ...
    double(include_ref_inp) * i.SIZE, ...
    control_weights_size, ...
    dyn_conf.get_params_size());
