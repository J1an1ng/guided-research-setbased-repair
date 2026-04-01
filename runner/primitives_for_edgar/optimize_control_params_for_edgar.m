%% Model settings
model = @kst;
k = KSTStateOrder();
inp_order = InpOrderADeltaV();
model_for_ref_traj = prevent_backward_driving(model);
%% TODO review params for Edgar
params = create_kst_params_edgar();
constraints = KSTConstraints(k, inp_order, params);
terminal_cond = StandstillTerminalCondition(k, 1);

%% Disturbances
state_meas_dist = zeros(k.SIZE, 1);
non_zero_meas_dist = meas_dist_for_kst_final(k);
%% TODO specify other disturbances if required
model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_final(k);

%% Out dir for checkpoint file
result_dir = ResultDir('out/opt_control_params_for_edgar/v2');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
control_params = [9.5192, 378.4692, 22.3658, 19.7692, 8.3620];
control_params_lb = [0, 0, 0, 19.7692, 8.3620];
control_params_ub = [100, 500, 100, 19.7692, 8.3620];

ref_state_builder = SameRefStateBuilder(k);
dyn_conf = ControlledModelParamConfig(false, ...
    k, inp_order, ...
    false, ... % we integrate the measurement uncertainty in the state-processor
    true,  ...
    false, ... % we only us additive disturbances
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ...
    ref_state_builder, control_params);

inp_bounds = create_inp_bounds_for_a_deltav(inp_order, params);
check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, @pd_controller, ...
    k, inp_order, params, dyn_conf, state_proc, check_inp);

create_dynamics = @() nonlinearSys('edgar_closed_loop_system', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

%% Further settings
dt_controller = 0.01;

simulator = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);

%% Initial sets that primitives for optimization start from
initial_sizes = dictionary();
initial_sizes(k.X) = 4.0 * non_zero_meas_dist(k.X);
initial_sizes(k.Y) = 5.0 * non_zero_meas_dist(k.Y);
initial_sizes(k.DELTA) = 50.0 * non_zero_meas_dist(k.DELTA);
initial_sizes(k.V) = 10.0 * non_zero_meas_dist(k.V);
initial_sizes(k.PSI) = 10.0 * non_zero_meas_dist(k.PSI);
interval_sampler = create_coarse_initial_sampler_for_truck_only( ...
    result_dir, 1, k, params, constraints, terminal_cond, initial_sizes);

%% Reference trajectories for primitives for optimization
primitive_duration = 1.0;
target_point_distance = 1.0;
% accs = {-2.0, 0.0, 1.5};
accs = {-1.0 };

pp_time_constant = 0.05;
inp_creator = InpCreatorForADeltaV(k, inp_order, params, pp_time_constant);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, inp_order, params, ...
    constraints, inp_creator, ...
    model, primitive_duration, accs, target_point_distance, false);

valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    primitive_duration, dt_controller, 2, model, k, inp_order, params, constraints);

[batch_idx, initial_sets] = interval_sampler.next_batch();

trajs = cell(0, 1);
counter = 1;
for i = 1:length(initial_sets)
    initial_set = initial_sets{i};
    [trajectories, ~] = valid_inp_traj_gen.generate(initial_set.c);
    trajs(counter:counter + length(trajectories) - 1) = trajectories(:);
    counter = counter + length(trajectories);
end

trajs = [trajs{:}];

%% -----------------------------------------------------------------------------------------------------
%% Optimization
%% -----------------------------------------------------------------------------------------------------
initial_itv_rad = zeros(k.get_size(), 1);
for i = 1:k.get_size()
    initial_itv_rad(i, 1) = initial_sizes(i);
end

cost_def = SimRefPathDeviationCost([k.Y], [1.0]);
% sim_sampler = RandomSimSampler(state_meas_dist, model_add_dist, []);
sim_sampler = WorstCaseSimSamplerForKST(k, inp_order, state_meas_dist, model_add_dist, []);
empty_dict = configureDictionary("string", "double");
opt = SimBasedControlWeightOptimizer(trajs, simulator, sim_sampler, ...
    dt_controller, ...
    20, initial_itv_rad, ...
    control_params_lb, control_params_ub, ...
    empty_dict, empty_dict, ...
    check_point_file_path, cost_def, true);
opt_ctrl_params = opt.optimize();
display(opt_ctrl_params);
%% -----------------------------------------------------------------------------------------------------

%% Result:
%%
