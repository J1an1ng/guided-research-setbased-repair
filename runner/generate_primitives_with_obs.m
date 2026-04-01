%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_1();

%% FIXME
% state_meas_dist = zero_meas_dist_for_kst_trailer(k);
% inp_dist = zero_inp_dist_for_a_deltav(i);
state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

out_path = 'out/obs_6';

num_workers = 8;

% interval_sampler = create_initial_interval_sampler_for_kst_trailer_1(k, params, constraints);
interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr( ...
    out_path, num_workers, k, params, constraints, terminal_cond);

%% Set up dynamics
ref_state_builder = SameRefStateBuilder(k);
% control_params = [37.5; 1200; 16.5; 80; 126];
% control_params = [21.0938; 398.4375; 10.5469; 194.5312; 84.3750];
control_params = [29.3493; 615.8613; 19.7979; 147.9185; 90.8707; 1.4171];

ref_state_size = ref_state_builder.get_ref_state_size();
create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, @pd_controller_2, ...
    k, i, params, ref_state_size), 3 * k.SIZE + ref_state_size + length(control_params), ...
    i.SIZE);

dt_controller = 0.01;

primitive_min_duration = 0.5; % 0.7; % 0.5;
primitive_max_duration = 0.75; % 1.0; % 0.6;
desired_duration = primitive_max_duration + 0.1;
target_point_distance = 1;
accs = [params.a_min, 0];
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;

occ_comp = TruckOccComputer(k, params);

debug = false;
create_plot = false;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, norm_final_set, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 0.1, ...
    control_params, ...
    terminal_cond, ...
    primitive_min_duration, primitive_max_duration, ...
    true, false, ... % FIXME set to true at some point
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, do_random_simulations);

gen = RepeatingReachPerformer( ...
    state_processor_class, ref_state_builder);

set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj, out_path);

set_gen.start_generation(num_workers);
