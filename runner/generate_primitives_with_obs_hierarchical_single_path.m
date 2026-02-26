%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTTrailerMirroring(k);
model_for_ref_traj = prevent_backward_driving(model);
params = create_kst_trailer_params_2();

state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/obs_hierarchical_single_path_starting_from_large_set_2');

%% Set up dynamics
ref_state_builder = SameRefStateBuilder(k);
[weights_size, f] = create_pd_control_weights_fun_with_increase_speed_wise(k);
dyn_conf = ControlledModelParamConfig(k, i, ...
    [], false, ... % we integrate the measurement uncertainty in the state-processor
    [], false, ...
    inp_dist, true, ...
    ref_state_builder, true, f, weights_size);

create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, @pd_controller, ...
    k, i, params, dyn_conf), 3 * k.SIZE, dyn_conf.get_params_size());

num_workers = 8;

base_interval_sampler = create_base_initial_sampler_for_hierarchical_single_path( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

dt_controller = 0.01;

primitive_duration = 1.0;
target_point_distance = inf; % 1.0

acc_max = -3.0;
accs = {acc_max};
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    primitive_duration, accs, target_point_distance, 0.01);

v_hierarchy_offset = 0.1;

abs_dims_k = KSTTrailerStateOrderOnlyAbs();
abs_dims = abs_dims_k.create_state_order_mapping(k);
maximum_relative_growth = 0.1;
get_max_rads = create_get_max_rads(k, abs_dims_k, constraints, state_meas_dist(k.DELTA));
check_size_soft = create_soft_size_checker(get_max_rads, abs_dims, maximum_relative_growth);
merger = SetMerger(abs_dims, get_max_rads, maximum_relative_growth, check_size_soft);

% this is the minimum velocity difference that each primitive reaches between
% the start and the end
hierarchical_dim_min_step_size = acc_max * primitive_duration;

state_space_divider = AdaptiveHierarchicalStateSpaceDivider(result_dir, ...
    k.V, hierarchical_dim_min_step_size, v_hierarchy_offset, ...
    base_interval_sampler, merger);

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;

occ_comp = TruckOccComputer(k, params);

debug = true;
create_plot = true;
show_plot = false;
save_plot = true;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ... % FIXME set to true at some point
    state_space_divider, ...
    occ_comp, ...
    debug, create_plot, show_plot, save_plot, do_random_simulations);

sim = StandardSimulator(create_dynamics(), dyn_conf);
% sim = WorstCaseKSTYSimulator(create_dynamics(), dyn_conf, state_meas_dist, [], inp_dist, k, i);
gen = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj);

set_gen.start_generation(num_workers);
