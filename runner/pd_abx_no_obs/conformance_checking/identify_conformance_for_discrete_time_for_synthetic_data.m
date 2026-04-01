%% System dynamics
model = @kst_trailer_inp_delta;
dt_controller = 0.01;
time_decimal_places = 2;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
veh_params = create_kst_trailer_params_MAN();

%% FIXME: these are the actual initial guesses:
state_meas_dist_non_zero = meas_dist_for_kst_trailer_inp_delta_1(k);
% model_add_dist = discr_time_kst_trailer_inp_delta_add_dist_1(k);
%% ...but they result in large reachsets, thus we take the zero dists
state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k); % zero_meas_dist_for_kst_trailer_inp_delta(k); %
model_add_dist = discr_time_kst_trailer_inp_delta_add_dist_1(k); % zero_kst_trailer_inp_delta_add_dist(k); %

%% The original gains for longitudinal PD controller result in large inputs partially:
control_params = [0.036, 0.72, 55.7358, 79.7620];

ref_state_builder = SameRefStateBuilder(k);
ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(true, ...
    k, i, ...
    true, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    state_meas_dist, model_add_dist, [], ...
    ref_state_builder, control_params);

[system_dynamics, system_output] = create_time_discrete_controlled_model(dt_controller);
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    @pd_controller_abx, ...
    k, i, veh_params, dyn_conf);

dyn = nonlinearSysDT('discr_time_kst_trailer_with_inp_delta', ...
    system_dynamics_wrapper, dt_controller, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, k.get_size());

result_dir = ResultDir('out/pd_abx_no_obs/conformance_checking');

%% Create reference trajectory - to be replaced with the one used during the conformance checking test drive
desired_duration = 1.0;
accs = {-1.5};
target_point_distance = 2.0;

constraints = KSTTrailerInpDeltaConstraints(k, i, veh_params);
terminal_cond = StandstillTerminalCondition(k, 1);
inp_creator = InpCreatorForADelta(k, i, veh_params);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, veh_params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, true);
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, dt_controller, time_decimal_places, model, k, i, veh_params, constraints);

options = create_cora_options_1();

%% Hint: this describes the expected uncertainty of the true initial states from the initial ref state,
%% which is not the same as the measurement uncertainty
initial_itv_sizes = state_meas_dist;

%% Initial estimates of disturbances:
n_k = 9;
params_for_conform.R0 = zonotope([zeros(k.get_size(), 1) eye(k.get_size()) ones(k.get_size(), 1)]);
params_for_conform.U = dyn_conf.create_scaffold_for_param_uncertainties(dyn_conf_values);
params_for_conform.tFinal = n_k * dt_controller;

%% FIXME continue here
[sampler, ~] = create_coarse_initial_sampler_for_kst_trailer_inp_delta( ...
    result_dir, 1, k, veh_params, constraints, terminal_cond, state_meas_dist);

params_for_conform.testSuite = generate_synthetic_test_suite(sampler, valid_inp_traj_gen, dyn, ...
    dyn_conf, dyn_conf_values, 5, dt_controller, n_k, initial_itv_sizes);

options_for_conform.zonotopeOrder = options.zonotopeOrder;
options_for_conform.tensorOrder = options.tensorOrder;
options_for_conform.errorOrder = options.errorOrder;
options_for_conform.tensorOrderOutput = options.tensorOrderOutput;
options_for_conform.cs.robustness = 1e-9;
options_for_conform.cs.verbose = false;
options_for_conform.cs.cost = "interval";
options_for_conform.cs.constraints = "gen";

identified_dists = conform(dyn, params_for_conform, options_for_conform, "white");

%% We perform the reachability analysis with the identified disturbances:
%% TODO

% reachsets = reach(dyn, params, options);

%% Plot result
f = figure;
cols = 3;
tiledlayout(cols, ceil(k.get_size() / cols));

for i = 1:k.get_size()
    nexttile;
    if ~isempty(reachsets)
        plotOverTime(reachsets, i, 'LineWidth', 3.0);
    end

    hold on;
    plot(ref_traj.times, ref_traj.states(i, :), 'Color', 'yellow');

    title(sprintf('%d', i));
end

fprintf('Done.\n');
