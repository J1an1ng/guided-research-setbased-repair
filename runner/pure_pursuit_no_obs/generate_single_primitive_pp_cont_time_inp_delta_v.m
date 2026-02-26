%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
mirroring = KSTTrailerMirroring(k);
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

%% FIXME update to finally used uncertainties
state_meas_dist = meas_dist_for_kst_trailer_final(k);
model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_trailer_final(k);

%% FIXME we specify the initial sizes based on the measurement uncertainties for now,
%% but this needs to be changed based on the reachability analysis for the nominal input
initial_sizes = dictionary();
initial_sizes(k.X) = 2 * state_meas_dist(k.X);
initial_sizes(k.Y) = 4.0 * state_meas_dist(k.Y);
initial_sizes(k.DELTA) = 2 * state_meas_dist(k.DELTA);
initial_sizes(k.V) = 2 * state_meas_dist(k.V);
initial_sizes(k.PSI) = 2 * state_meas_dist(k.PSI);
initial_sizes(k.HA) = 2 * state_meas_dist(k.HA);

%% FIXME
%% state_meas_dist(k.DELTA) = 0;

%% FIXME remove
% state_meas_dist = zero_meas_dist_for_kst_trailer(k);

constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/pure_pursuit_no_obs/single_primitive_inp_delta_v');

primitive_duration = 3.0;
desired_duration = primitive_duration;
target_point_distance = 0.1;

accs = {-0.5, -1.0, -1.5};

pp_time_constant = 0.05;

inp_creator = InpCreatorForADeltaV(k, i, params, pp_time_constant);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, false);

pure_pursuit_lookahead_time = 0.3;
dt_controller = 0.01;

pure_pursuit_lookahead_offset = 0.3;
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, dt_controller, 2, model, k, i, params, constraints, ...
    pure_pursuit_lookahead_offset, @process_final_state_for_ext_horizon);

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

%% Set up dynamics
ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [0.1, 19.7692, 8.3620]; % 55.7358, 79.7620];
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
    state_proc.get_state_size(), dyn_conf.get_params_size()); % , ...
% system_output, k.get_size());

num_workers = 1;
interval_sampler = create_base_initial_sampler_for_hierarchical( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, initial_sizes);

occ_comp = create_kst_trailer_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = true;
do_random_simulations = true;

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
    primitive_duration, ...
    false, false, ...
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, true, false, do_random_simulations, ...
    pure_pursuit_lookahead_offset, @process_final_state_for_ext_horizon);

[batch_idx, initial_sets] = interval_sampler.next_batch();

%% initial_set = initial_sets{150}; %% FIXME
initial_set = initial_sets{1};

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, valid_inp_traj_gen);

reach_performer = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values);

% figure;
% hold on;
% for i = 1:length(p)
%     pr = p{i};
%     plot(pr.ref_traj.states(1, :), pr.ref_traj.states(2, :));
% end

% fprintf('Number of primitives: %d\n', length(p));

% p = p(1);
% p{1}.set_final_set_id(10);
% q = parallel.pool.DataQueue;
% p{1} = perform_reach_for_primitive(p{1}, settings, ...
%   reach_performer, dictionary(uint32.empty, uint32.empty), q);

p = p{1};
dyn = create_dynamics();

log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
log_fid = fopen(log_file_path, 'wt');

res = reach_performer.generate(settings, p, dyn, log_fid);

sim_res = [];
if settings.create_plot && settings.do_random_simulations
    sim = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);
    sim_sampler = LateralWCSimSampler(k, i, state_meas_dist, model_add_dist, []);
    initial_itv_rads = zeros(k.get_size(), 1);
    initial_itv_rads(1:k.get_size()) = initial_sizes(1:k.get_size());
    sim_res = sim.simulate(100, p.ref_traj, initial_itv_rads, dt_controller, [], sim_sampler);
end

if settings.create_plot
    plot_primitive(settings, p, res.reachsets, sim_res);
    plot_occupancy_and_save(settings, p, res.reachsets, 10);
end

create_occupancy_plot(occ_comp, k, params, initial_set.c, p.ref_traj, res, max_half_width, 10);

ylim([-3, 3]);
