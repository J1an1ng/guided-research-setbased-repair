%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderAroc;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_1();

%% FIXME
state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr(k, params, constraints);

%% Set up dynamics
create_dynamics = @() nonlinearSys(@(x, u) model(x, u, k, i, params), k.SIZE, 2);
%% FIXME i.SIZE instead of 2??

dt_controller = 0.01;

primitive_min_duration = 0.6;
primitive_max_duration = 0.6;
desired_duration = primitive_max_duration + 0.1;
target_point_distance = 1;
accs = [params.a_min, 0];
terminal_cond = StandstillTerminalCondition(k, 1);
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);

occ_comp = TruckOccComputer(k, params);

out_path = 'out/aroc';
save_to_mat = false;
export_to_cr = true;
num_primitives_per_file = 2;
debug = true;
create_plot = true;
do_random_simulations = false;

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
    debug, create_plot, do_random_simulations);

gen = ArocPrimitiveGenerator(10);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj, [], out_path);

set_gen.start_generation_sequentially();
% set_gen.start_generation_parallel();
