%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shiftFinalSet_truck_default;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_1();
state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
interval_sampler = create_initial_interval_sampler_for_kst_trailer_1(k, params, constraints);

%% Set up dynamics
create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model, model, @pd_controller, ...
    k, i, params), 3 * k.SIZE, 2);

dt_controller = 0.01;
primitive_min_duration = 0.5;
primitive_max_duration = 0.6;
inp_traj_gen = KSTConstatntInpTrajGenerator(k, i, [0 params.steering_v_max], 0.05, [params.a_min, 0], 1);

state_processor_class = @StateProcessorForDiscretelyControlled;

out_path = 'out/no_obs';
num_primitives_per_file = 1;
debug = 1;
create_plot = 1;
do_random_simulations = 1;

gen = PrimitiveSetGenerator( ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, ...
    state_meas_dist, inp_dist, ...
    inp_traj_gen, primitive_min_duration, primitive_max_duration, ...
    dt_controller, model_for_ref_traj, ...
    state_processor_class, ...
    out_path, num_primitives_per_file, debug, ...
    create_plot, do_random_simulations, ...
    interval_sampler);

gen.start_generation_sequentially();
% gen.start_generation_parallel(16);
