%% Model settings
model = @kst;
k = KSTStateOrder();
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTMirroring(k);
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_params_MAN();

state_meas_dist = meas_dist_for_kst_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);

constraints = KSTConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/dummy_8_for_truck_only_smaller_v_range_4_smaller_a_min_lat_4');

primitive_duration = 1.0;
desired_duration = primitive_duration;
target_point_distance = 0.05;

accs = {-0.5, -1.0, -1.5};
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);

num_workers = 8; %% FIXME

interval_sampler = create_base_initial_sampler_for_hierarchical_for_truck_only( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

create_dynamics = @() [];

dt_controller = 0.01;

occ_comp = create_kst_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = false;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, ...
    max_half_width, ...
    constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ...
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, false, false, do_random_simulations);

traj_dummy_set = interval(-state_meas_dist, state_meas_dist);
% we reduce the final interval size as the normalization increase the
% size in some dimensions again
final_dummy_set = 0.1 * interval(-state_meas_dist, state_meas_dist);

gen = DummyReachPerformer(traj_dummy_set, final_dummy_set);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj);

set_gen.start_generation(num_workers);
