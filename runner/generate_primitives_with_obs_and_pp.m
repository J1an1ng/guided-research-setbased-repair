%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_1();

%% FIXME
state_meas_dist = zero_meas_dist_for_kst_trailer(k);
inp_dist = zero_inp_dist_for_a_deltav(i);
% state_meas_dist = meas_dist_for_kst_trailer_1(k);
% inp_dist = inp_dist_for_a_deltav_1(i);

constraints = KSTTrailerConstraints(k, i, params);
interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr(k, params, constraints);

dt_controller = 0.01;
pure_pursuit_lookahead_time = 0.2;
pure_pursuit_lookahead_num_steps = pure_pursuit_lookahead_time / dt_controller;

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

%% Set up dynamics
%% TODO: maybe decrease over time as measurement
%% becomes more accurate??
ref_state_builder = LookaheadRefStateBuilder(k, pure_pursuit_lookahead_num_steps, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [0.02, 50, 50, 1];
ref_state_size = ref_state_builder.get_ref_state_size();
create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, ...
    create_lat_pure_pursuit_long_pd(reduced_k, position_k), ...
    k, i, params, ref_state_size), 3 * k.SIZE + ref_state_size + length(control_params), ...
    i.SIZE);

primitive_min_duration = 0.5; % 0.7; % 0.5;
primitive_max_duration = 0.75; % 1.0; % 0.6;
desired_duration = primitive_max_duration + 0.1;
target_point_distance = 1;
accs = [params.a_min, 0];
terminal_cond = StandstillTerminalCondition(k, 1);
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;

occ_comp = TruckOccComputer(k, params);

out_path = 'out/obs_pp';
save_to_mat = false;
export_to_cr = true;
num_primitives_per_file = 2;
debug = true;
create_plot = true;
do_random_simulations = true;

settings = PrimitiveGenerationSettings( ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, ...
    state_meas_dist, inp_dist, ...
    dt_controller, ...
    terminal_cond, ...
    primitive_min_duration, primitive_max_duration, ...
    true, ...
    interval_sampler, ...
    save_to_mat, export_to_cr, num_primitives_per_file, ...
    occ_comp, ...
    debug, create_plot, do_random_simulations, ...
    pure_pursuit_lookahead_time, @process_final_state_for_ext_horizon);

gen = RepeatingReachPerformer( ...
    state_processor_class, ref_state_builder);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj, control_params, out_path);

set_gen.start_generation_sequentially();
% set_gen.start_generation_parallel();
