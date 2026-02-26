%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTTrailerMirroring(k);
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_2();

state_meas_dist = zero_meas_dist_for_kst_trailer(k); %% FIXME
% inp_dist = zero_inp_dist_for_a_deltav(i);
% state_meas_dist = meas_dist_for_kst_trailer_1(k); %% FIXME
inp_dist = inp_dist_for_a_deltav_1(i);
model_dists = model_dist_for_kst_trailer_1(k);
constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/single_primitive_cont_3');

ref_state_builder = SameRefStateBuilder(k);
%% control_params = [37.5; 1200; 16.5; 80; 126];
weights_size = 5;
weights = [34.4; 1187.5; 23.4; 46.9; 162.5];
% control_params = [21.0938; 398.4375; 10.5469; 194.5312; 84.3750];
% control_params = [10.9375; 343.7500; 17.9688; 257.8125; 68.7500];

dyn_conf = ControlledModelParamConfig(k, i, ...
    state_meas_dist, true, ...
    [], false, ...
    inp_dist, true, ...
    ref_state_builder, true, weights, weights_size);

create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @continuously_controlld_model, model, @pd_controller, ...
    k, i, params, dyn_conf), k.SIZE, dyn_conf.get_params_size());

dt_controller = 0.01;

primitive_duration = 0.5; % 0.75
desired_duration = primitive_duration + 0.1;
target_point_distance = 1.0;

acc_max = -3.0;
accs = acc_max;
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);

num_workers = 1;

itv = zeros(k.SIZE, 2);
itv(k.X, :) = [-state_meas_dist(k.X), state_meas_dist(k.X)];
itv(k.Y, :) = [-2 * state_meas_dist(k.Y), 2 * state_meas_dist(k.Y)];

% TODO check different speeds
itv(k.V, :) = [params.v_max - 1., params.v_max];
v_mean = (itv(k.V, 1) + itv(k.V, 2)) / 2;

get_delta = create_get_delta_range_for_v(constraints, state_meas_dist(k.DELTA));

% itv(k.DELTA, :) = 4 * [-state_meas_dist(k.DELTA), state_meas_dist(k.DELTA)];
delta_range = get_delta(v_mean);
fprintf('Delta range: %f\n', delta_range);
itv(k.DELTA, :) = [-delta_range / 2, delta_range / 2];

itv(k.PSI, :) = [-state_meas_dist(k.PSI), state_meas_dist(k.PSI)];

itv(k.HA, :) = [-0.0015, 0.0015];
itv = interval(itv(:, 1), itv(:, 2));
initial_set = SetContainer(1, itv, true);

occ_comp = TruckOccComputer(k, params);

debug = true;
create_plot = true;
show_plot = true;
do_random_simulations = false;

dummy_state_space_divider = create_base_initial_sampler_for_hierarchical_single_path( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, model_dists, ...
    dt_controller, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ...
    dummy_state_space_divider, ...
    occ_comp, ...
    debug, create_plot, show_plot, false, do_random_simulations);

traj_gen = ValidInpTrajGenerator(inp_traj_gen, settings.terminal_cond, ...
        settings.primitive_duration, ...
        settings.dt_control, ...
        settings.time_decimal_places, ...
        model_for_ref_traj, settings.k, settings.i, params, ...
        settings.constraints, ...
        settings.extended_horizon, settings.process_final_state);

% sim = WorstCaseKSTYSimulator(create_dynamics(), state_meas_dist, inp_dist, k, i);
sim = StandardSimulator(create_dynamics(), dyn_conf);

reach_performer = OneReachPerformer(dyn_conf, sim);

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");
mkdir(sub_out_path);

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, traj_gen);

fprintf('Number of primitives: %d\n', length(p));

p = p(2);

p{1}.set_final_set_id(10);
q = parallel.pool.DataQueue;
p{1} = perform_reach_for_primitive(p{1}, settings, ...
  reach_performer, dictionary(uint32.empty, uint32.empty), q);

if ~p{1}.is_valid()
    error('Primitive is not valid');
end

figure;
p{1}.plot_occupancy(occ_comp, 10);
