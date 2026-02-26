%% -----------------------------------------------------------------------------------------------------
%% Model settings
%% -----------------------------------------------------------------------------------------------------
model = @kst_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
model_for_ref_traj = prevent_backward_driving(model);

%% -----------------------------------------------------------------------------------------------------
%% Configs
%% -----------------------------------------------------------------------------------------------------
veh_params = create_kst_trailer_params_MAN_v2();
state_meas_dist = meas_dist_for_kst_trailer_final(k);
model_add_dist = model_dist_for_kst_trailer_final(k);

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
duration = 1.0;
time_decimal_places = 2;

constraints = KSTTrailerConstraints(k, i, veh_params);

result_dir = ResultDir('out/opt_lat_control_pp_no_obs_final_2');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
long_params = [19.7692, 8.3620];
control_params = [0.05, long_params];

ref_state_args = dictionary();
ref_state_args("pure_pursuit_lookahead_num_steps") = double(uint32(0.2 / dt_controller));

control_params_lb = [0.01, long_params];
control_params_ub = [0.5, long_params];

min_pure_pursuit_lookahead_time = 0.1;
max_pure_pursuit_lookahead_time = 0.5;
ref_state_args_lb = dictionary();
ref_state_args_lb("pure_pursuit_lookahead_num_steps") = double(uint32(min_pure_pursuit_lookahead_time / dt_controller));
ref_state_args_ub = dictionary();
ref_state_args_ub("pure_pursuit_lookahead_num_steps") = double(uint32(max_pure_pursuit_lookahead_time / dt_controller));

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));

ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ref_state_builder, [], ref_state_args);

inp_bounds = create_inp_bounds_for_a_deltav(i, veh_params);
check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, ~] = create_cont_time_discr_contr_model( ...
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k), ...
    k, i, veh_params, dyn_conf, state_proc, check_inp);

create_dynamics = @() nonlinearSys('my_final_model', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

simulator = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);

%% -----------------------------------------------------------------------------------------------------
%% Generation of reference trajectories
%% -----------------------------------------------------------------------------------------------------

%% Coarse initial sets covering roughly the entire state space
initial_sizes = dictionary();
initial_sizes(k.X) = 2 * state_meas_dist(k.X);
initial_sizes(k.Y) = 4 * state_meas_dist(k.Y);
initial_sizes(k.DELTA) = 2 * state_meas_dist(k.DELTA);
initial_sizes(k.V) = 2 * state_meas_dist(k.V);
initial_sizes(k.PSI) = 2 * state_meas_dist(k.PSI);
initial_sizes(k.HA) = 2 * state_meas_dist(k.HA);
interval_sampler = create_coarse_initial_sampler( ...
    result_dir, 1, k, veh_params, constraints, terminal_cond, initial_sizes);

%% Reference trajectories
target_point_distance = 0.3;
accs = {-1.5};

pp_time_constant = 0.05;
inp_creator = InpCreatorForADeltaV(k, i, veh_params, pp_time_constant);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, veh_params, ...
    constraints, inp_creator, ...
    model, duration, accs, target_point_distance, false);

pure_pursuit_lookahead_offset = 0.1;
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    duration, dt_controller, 2, model, k, i, veh_params, constraints, ...
    pure_pursuit_lookahead_offset, @process_final_state_for_ext_horizon);

[batch_idx, initial_sets] = interval_sampler.next_batch();

trajs = cell(0, 1);
counter = 1;
for i = 1:length(initial_sets)
    initial_set = initial_sets{i};
    [trajectories, ~] = valid_inp_traj_gen.generate(initial_set.c);
    trajs(counter:counter + length(trajectories) - 1) = trajectories(:);
    counter = counter + length(trajectories);
end

trajs = [trajs{:}];

%% -----------------------------------------------------------------------------------------------------
%% Optimization
%% -----------------------------------------------------------------------------------------------------
initial_itv_rad = zeros(k.get_size(), 1);
for i = 1:k.get_size()
    initial_itv_rad(i, 1) = initial_sizes(i);
end

cost_def = SimRefPathDeviationCost([k.Y], [1.0]);
sim_sampler = RandomSimSampler(state_meas_dist, model_add_dist, []);
opt = SimBasedControlWeightOptimizer(trajs, simulator, sim_sampler, ...
    dt_controller, ...
    10, initial_itv_rad, ...
    control_params_lb, control_params_ub, ...
    ref_state_args_lb, ref_state_args_ub, ...
    check_point_file_path, cost_def, false);
opt_ctrl_params = opt.optimize();
display(opt_ctrl_params);
%% -----------------------------------------------------------------------------------------------------

%% Result:
%% time constant: 0.9691
%% lookahead num steps: 97
%% Result with new long params:
%% time constant: 0.9603
%% lookahead num steps: 76.79
%% Result with new long params and changed lookahead offset:
%% time constant: 0.4976
%% lookahead num steps: 48
