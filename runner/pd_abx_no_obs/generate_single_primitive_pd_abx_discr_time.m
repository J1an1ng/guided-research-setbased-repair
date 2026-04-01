%% Model settings
model = @kst_trailer_inp_delta;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
mirroring = NoMirroring();
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN();

state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k); % zero_meas_dist_for_kst_trailer_inp_delta(k);
model_add_dist = meas_dist_for_kst_trailer_inp_delta_2_small(k); % zero_kst_trailer_inp_delta_add_dist(k);

constraints = KSTTrailerInpDeltaConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/pd_abx_no_obs/single_primitive');

primitive_duration = 1.0;
desired_duration = primitive_duration;
target_point_distance = 0.1;

accs = {-0.5, -1.0, -1.5};

inp_creator = InpCreatorForADelta(k, i, params);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, true);

valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, 0.01, 2, model, k, i, params, constraints);

%% Set up dynamics
%% TODO: maybe decrease over time as measurement
%% becomes more accurate??
ref_state_builder = SameRefStateBuilder(k);
%% control_params = [0.036, 0.72, 55.7358, 79.7620];
%% control_params = [0.036, 0.72, 7.6989, 2.4944];
control_params = [0.036, 0.72, 4.9, 2.4];

%% The first two weights are from abx controller and multiplied by the wheelbase as he controls curvature
%% and not steering angle
ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(true, ...
    k, i, ...
    true, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

load('out/cc_data/cc_result_pp_6'); %% cc_result_6
% combined_dist_set = meas_dist_and_model_add_dist;
combined_dist_set = meas_dists_and_add_dists;

dyn_conf_values = ControlledModelParamValues( ...
    state_meas_dist, model_add_dist, [], ...
    combined_dist_set, ...
    ref_state_builder, control_params);

dt_controller = 0.01;

[system_dynamics, system_output] = create_time_discrete_controlled_model(dt_controller);
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    @pd_controller_abx, ...
    k, i, params, dyn_conf);

create_dynamics = @() nonlinearSysDT('discr_time_kst_trailer_with_inp_delta', ...
    system_dynamics_wrapper, dt_controller, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, k.get_size());

num_workers = 1;
interval_sampler = create_base_initial_sampler_for_hierarchical_for_inp_delta( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

state_processor_class = @StandardStateProcessor;

occ_comp = create_kst_trailer_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = true;
do_random_simulations = false; %% FIXME

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, ...
    max_half_width, ...
    constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, [], model_add_dist, ...
    dt_controller, 2, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ...
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, true, false, do_random_simulations);

[batch_idx, initial_sets] = interval_sampler.next_batch();

initial_set = initial_sets{1};

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, valid_inp_traj_gen);

% sim = WorstCaseKSTInpDetlaForYSimulator(create_dynamics(), dyn_conf, state_meas_dist, model_add_dist, [], k, i);
sim = StandardSimulator(create_dynamics(), dyn_conf);

reach_performer = OneReachPerformer(state_processor_class, dyn_conf, dyn_conf_values, sim);

% reach_performer = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

figure;
hold on;
for i = 1:length(p)
    pr = p{i};
    plot(pr.ref_traj.states(1, :), pr.ref_traj.states(2, :));
end

fprintf('Number of primitives: %d\n', length(p));

p = p(1);
p{1}.set_final_set_id(10);
q = parallel.pool.DataQueue;
p{1} = perform_reach_for_primitive(p{1}, settings, ...
  reach_performer, dictionary(uint32.empty, uint32.empty), q);
