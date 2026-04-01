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

[system_dynamics, system_output] = create_time_discrete_controlled_model(dt_controller, k);
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    create_lat_pure_pursuit_long_pd_for_inp_delta(reduced_k, position_k), ...
    k, i, veh_params, dyn_conf);

out_size = k.get_size() - 1; %% FIXME just tmply reduced by 1 for removing ha

dyn = nonlinearSysDT('discr_time_kst_trailer_with_inp_delta_pp', ...
    system_dynamics_wrapper, dt_controller, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, out_size);

%% Load recorded data
rec_data_file_path = 'out/cc_data/record_10';
rec_data = load(rec_data_file_path);
rec_data = rec_data.res;

num_rec_steps = size(rec_data.output.x, 2);
num_steps = num_rec_steps - pure_pursuit_lookahead_num_steps - 10;

ref_times = 0:dt_controller:num_steps * dt_controller;

ref_states = zeros(k.get_size(), num_rec_steps);
ref_states(k.X, :) = rec_data.ref.x;
ref_states(k.Y, :) = rec_data.ref.y;
ref_states(k.V, :) = rec_data.ref.v;
ref_states(k.PSI, :) = rec_data.ref.psi;

ref_inputs = zeros(i.SIZE, num_steps);

ref_traj = PredefinedTraj(ref_times, ref_inputs, ref_states, false, []);

[u, ~] = dyn_conf.create_params_for_random_sim(ref_traj, 1:num_steps, dyn_conf_values);

simParams.tStart = 0;
simParams.tFinal = num_steps * dt_controller;
simParams.x0 = ref_states(:, 1);
simParams.u = u;

[t_res, x_res] = simulate(dyn, simParams);

f = figure;
cols = 2;
tiledlayout(cols, 2); % ceil(k.get_size() / cols));

for dim = 1:4 % k.get_size()
    nexttile;

    plot(x_res(:, dim), 'Color', 'green');
    hold on;
    ll = lower(k.label_from_state_idx(dim));
    plot(rec_data.output.(ll)(1:num_steps), 'Color', 'yellow');
    plot(rec_data.ref.(ll)(1:num_steps), 'Color', 'red');
    title(sprintf('%s', ll));
end
