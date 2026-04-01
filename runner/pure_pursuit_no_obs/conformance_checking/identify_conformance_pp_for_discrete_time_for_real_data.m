%% System dynamics
model = @kst_trailer_inp_delta;
dt_controller = 0.01;
time_decimal_places = 2;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
veh_params = create_kst_trailer_params_MAN();

state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k); % zero_meas_dist_for_kst_trailer_inp_delta(k); %
model_add_dist = discr_time_kst_trailer_inp_delta_add_dist_1(k); % zero_kst_trailer_inp_delta_add_dist(k); %

%% The original gains for longitudinal PD controller result in large inputs partially:
control_params = [4.9, 2.4];

pure_pursuit_lookahead_time = 1.0;
pure_pursuit_lookahead_num_steps = pure_pursuit_lookahead_time / dt_controller;

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

ref_state_builder = LookaheadRefStateBuilder(k, pure_pursuit_lookahead_num_steps, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(true, ...
    k, i, ...
    true, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    state_meas_dist, model_add_dist, [], ...
    [], ...
    ref_state_builder, control_params);

[system_dynamics, system_output] = create_time_discrete_controlled_model(dt_controller);
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    create_lat_pure_pursuit_long_pd_for_inp_delta(reduced_k, position_k), ...
    k, i, veh_params, dyn_conf);

out_size = k.get_size() - 1; %% FIXME just tmply reduced by 1 for removing ha

dyn = nonlinearSysDT('discr_time_kst_trailer_with_inp_delta_pp', ...
    system_dynamics_wrapper, dt_controller, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, out_size);

options = create_cora_options_1();

%% Initial estimates of disturbances:
n_k = 10;
params_for_conform.R0 = zonotope([zeros(k.get_size(), 1) eye(k.get_size())]); % ones(k.get_size(), 1)]);
params_for_conform.U = dyn_conf.create_scaffold_for_param_uncertainties(dyn_conf_values);

num_gens_R0 = size(params_for_conform.R0.generators, 2);
num_gens_U = size(params_for_conform.U.generators, 2);
num_gens = num_gens_R0 + num_gens_U;

params_for_conform.tFinal = n_k * dt_controller;

rec_id = 6;
rec_data_file_path = sprintf('out/cc_data/record_opt_2_%d', rec_id);
rec_data = load(rec_data_file_path);
rec_data = rec_data.res;

length_test_case = n_k;
margin_end = pure_pursuit_lookahead_num_steps + 1;
ext_horizon_time = pure_pursuit_lookahead_time; % only need for pure pursuit
params_for_conform.testSuite = create_test_suite_from_data( ...
    rec_data, dt_controller, ...
    class(dyn), length_test_case, margin_end, ...
    k, i, ext_horizon_time, dyn_conf, dyn_conf_values);

options_for_conform.zonotopeOrder = options.zonotopeOrder;
options_for_conform.tensorOrder = options.tensorOrder;
options_for_conform.errorOrder = options.errorOrder;
options_for_conform.tensorOrderOutput = options.tensorOrderOutput;
options_for_conform.cs.robustness = 1e-9;
options_for_conform.cs.verbose = false;
options_for_conform.cs.cost = "interval"; % "frob";
options_for_conform.cs.constraints = "gen";
w = ones(n_k + 1, 1);
w(end) = 10;
options_for_conform.cs.w = w;

a_max = ones(num_gens, 1);
a_max(:) = Inf;
a_max(num_gens_R0 + 3) = 0.03;
a_max(num_gens_R0 + 8) = 0.001;
options_for_conform.cs.a_max = a_max; %% FIXME why does this not work??

identified_dists = conform(dyn, params_for_conform, options_for_conform, "white");

meas_dist_and_add_model_dist = project(identified_dists.U, 1:10);
res_file_path = sprintf('out/cc_data/res/cc_result_opt_3_%d', rec_id);
save(res_file_path, 'meas_dist_and_add_model_dist');

return

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
