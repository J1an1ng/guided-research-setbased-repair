%% Model settings
model = @kst;
k = KSTStateOrder();
i = InpOrderADeltaV();
mirroring = KSTMirroring(k);
norm_final_set = @norm_kst;

model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

state_meas_dist = meas_dist_for_kst_final(k);
model_add_dist = model_dist_for_kst_final(k);

constraints = KSTConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/pure_pursuit_no_obs/simple_reach_truck');

primitive_duration = 1.0;
desired_duration = primitive_duration;
target_point_distance = 0.05;

accs = {-0.5, -1.0, -1.5, -2.0, -2.5};

pp_time_constant = 0.05;

inp_creator = InpCreatorForADeltaV(k, i, params, pp_time_constant);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, false);

num_workers = 8; %% FIXME

initial_sizes = dictionary();
initial_sizes(k.X) = state_meas_dist(k.X);
initial_sizes(k.Y) = 2.0 * state_meas_dist(k.Y);
initial_sizes(k.DELTA) = 2.0 * state_meas_dist(k.DELTA);
initial_sizes(k.V) = 2.0 * state_meas_dist(k.V);
initial_sizes(k.PSI) = state_meas_dist(k.PSI);
interval_sampler = create_base_initial_sampler_for_hierarchical_for_truck_only( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, initial_sizes);

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
    norm_final_set, ...
    mirroring, ...
    state_meas_dist, [], model_add_dist, ...
    dt_controller, 2, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ...
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, false, false, do_random_simulations);

constant_set = interval(-5.0 * state_meas_dist, 5.0 * state_meas_dist);
% we reduce the final interval size as the normalization increase the
% size in some dimensions again
final_constant_set = 0.1 * interval(-state_meas_dist, state_meas_dist);

gen = ConstantReachPerformer(constant_set, final_constant_set);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj);

set_gen.start_generation(num_workers);
