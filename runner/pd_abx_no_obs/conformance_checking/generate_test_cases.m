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

state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k);
inp_dist = inp_dist_for_a_delta_1(i);
add_dist = model_dist_for_kst_trailer_inp_delta_1(k);

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

dt_controller = 0.01;

%% Set up dynamics
%% TODO: maybe decrease over time as measurement
%% becomes more accurate??
ref_state_builder = SameRefStateBuilder(k);
control_params = [0.036, 0.72, 55.7358, 79.7620];
%% The first two weights are from abx controller and multiplied by the wheelbase as he controls curvature
%% and not steering angle
ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(k, i, ...
    state_meas_dist, true, ... %[], false, ... %% previously, we integrate the measurement uncertainty in the state-processor
    add_dist, true, ... % [], false, ...
    [], false, ...% inp_dist, true, ...
    ref_state_builder, true, control_params, length(control_params));

%% Hint: we changed the system to be continuously controlled to reduce the state size;
%% even though the real system is discretely controlled, this deviation is captured by
%% conformance checking.
create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @continuously_controlld_model, ... %@discretely_controlled_model, ...
    model, ...
    @pd_controller_abx, ...
    k, i, params, dyn_conf), k.get_size(), ... % 2*
    dyn_conf.get_params_size());

num_workers = 1;
interval_sampler = create_base_initial_sampler_for_hierarchical_for_inp_delta( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

state_processor_class = @StandardStateProcessor;

occ_comp = create_kst_trailer_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = true;
do_random_simulations = false; % true;

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
    debug, create_plot, true, false, do_random_simulations);

[batch_idx, initial_sets] = interval_sampler.next_batch();

%% TODO: for generating artificial test cases for checking the basic CC algorithm:
%% - create coarse initial set sampler
%% - for each set: generate primitives with higher resolution than currently
%% - do the WC-simulations

%% TODO: for generating reference trajectories for real test drives:
%% apply trajsampler to state with high speed; use primitive with certain acceleration and
%% a last state which is suitable (should be determined by road somehow...) ...then,
%% for the last state: normalize it (position to zero and heading to zero), apply again trajsampler...
%% the resulting reftraj then needs to be transformed back or maybe also not...depending on how the ABX is exactly working
%%

%% TODO: Optimizer: optimize for different params - they should scale the disturbances - how exactly is part of the obj function...

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, valid_inp_traj_gen);

sim = WorstCaseKSTInpDetlaForYSimulator(create_dynamics(), dyn_conf, state_meas_dist, add_dist, inp_dist, k, i);

reach_performer = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

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
