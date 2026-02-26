%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
mirroring = KSTTrailerMirroring(k);
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

state_meas_dist = 0.01 * meas_dist_for_kst_trailer_final(k);
state_meas_dist(k.DELTA) = 0;
model_add_dist = 0.01 * model_dist_for_kst_trailer_final(k);

constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/pure_pursuit_no_obs/full_fail_safe_traj');

%% Initial set
f = create_get_delta_rad_for_v(constraints, state_meas_dist(k.DELTA));

v = 15.0;
delta_size = f(v);
start_itv = zeros(k.get_size(), 2);
start_itv(k.X, :) = 2 * [-state_meas_dist(k.X), state_meas_dist(k.X)];
start_itv(k.Y, :) = 4 * [-state_meas_dist(k.Y), state_meas_dist(k.Y)];
start_itv(k.DELTA, :) = 2 * [-state_meas_dist(k.DELTA), state_meas_dist(k.DELTA)]; % [-delta_size, delta_size];
start_itv(k.V, :) = [v - 2 * state_meas_dist(k.V), v + 2 * state_meas_dist(k.V)];
start_itv(k.PSI, :) = 2 * [-state_meas_dist(k.PSI), state_meas_dist(k.PSI)];
start_itv(k.HA, :) = 2 * [-state_meas_dist(k.HA), state_meas_dist(k.HA)];
start_set = SetContainer(1, interval(start_itv(:, 1), start_itv(:, 2)), true);

%% Reference trajectory
a = -2.0;
pp_time_constant = 0.05;

inp_creator = InpCreatorForADeltaV(k, i, params, pp_time_constant);

total_duration = abs(v / a);
total_dist = abs((v^2) / (2 * a));

lane_width = 3.75;
shoulder_width = 2.75;
evasive_dist = lane_width / 2.0 + shoulder_width / 2.0;

a_fun = create_acc_function(a, total_duration / 2);

dist_1 = total_dist / 2.0;
duration_1 = (-v + sqrt(v^2 + 2 * a * dist_1)) / a;
duration_1 = 0.9 * duration_1; % for preventing "overshooting" in pure pursuit

traj_part_1 = KSTInpTrajTargetPoint(k, params, constraints, ...
    a, a_fun, [dist_1; -evasive_dist / 2.0], inp_creator);

offset_2 = 10.0;
traj_part_2 = KSTInpTrajTargetPoint(k, params, constraints, ...
    a, a_fun, [total_dist + offset_2; -evasive_dist], inp_creator);
traj = CombinedTraj({traj_part_1; traj_part_2}, [duration_1; total_duration]);

%% System dynamics
pure_pursuit_lookahead_time = 0.2;
dt_controller = 0.01;

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [0.05, 12.8747, 2.5546];
ref_state_size = ref_state_builder.get_ref_state_size();

ref_state_args = dictionary();
ref_state_args("pure_pursuit_lookahead_num_steps") = double(uint32(pure_pursuit_lookahead_time / dt_controller));

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, ... % we integrate the measurement uncertainty in the state-processor
    true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ...
    ref_state_builder, control_params, ref_state_args);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k), ...
    k, i, params, dyn_conf, state_proc);

create_dynamics = @() nonlinearSys('my_final_model', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

%% Settings
occ_comp = create_kst_trailer_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = true;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, ...
    max_half_width, ...
    constraints, ...
    norm_final_set, ...
    mirroring, ...
    state_meas_dist, [], model_add_dist, ...
    dt_controller, 2, 0.1, ...
    terminal_cond, ...
    [], ...
    false, false, ...
    [], ...
    occ_comp, ...
    debug, create_plot, true, false, do_random_simulations, ...
    pure_pursuit_lookahead_time, @process_final_state_for_ext_horizon);

%% We check the trajectory with the simulator
simulator = Simulator(terminal_cond, settings.time_decimal_places, ...
    model_for_ref_traj, k, i, params, constraints, ...
    pure_pursuit_lookahead_time, @process_final_state_for_ext_horizon);
success = simulator.simulate(traj, center(start_set.cora_set), total_duration, dt_controller);

if ~success
    %% ignore for now
    warning("Invalid trajectory");
    % error("Invalid trajectory");
end

%% We generate the primitive
gen = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values, []);
p = Primitive(start_set, traj, dt_controller, 0.1);

dyn = create_dynamics();

log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
log_fid = fopen(log_file_path, 'wt');

res = gen.generate(settings, p, dyn, log_fid);
save('out/pure_pursuit_no_obs/full_fail_safe_traj/reachsets_pkl', 'res');

% load('out/pure_pursuit_no_obs/full_fail_safe_traj/reachsets_pkl', 'res');
plot_primitive(settings, p, res.reachsets, []);

%% We create several plots
create_occupancy_plot(occ_comp, k, params, center(start_set.cora_set), p.ref_traj, res, max_half_width, 100);
