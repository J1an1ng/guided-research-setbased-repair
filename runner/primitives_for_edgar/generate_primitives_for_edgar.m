%% Model settings
model = @kst_inp_delta;
k = KSTInpDeltaStateOrder();
i = InpOrderADelta();
mirroring = NoMirroring();
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);
%% TODO review params for Edgar
params = create_kst_params_edgar();
constraints = KSTInpDeltaConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

%% Disturbances
state_meas_dist = zeros(k.get_size(), 1);
non_zero_meas_dist = zeros(k.get_size(), 1);
non_zero_meas_dist(k.X) = 0.03;
non_zero_meas_dist(k.Y) = 0.01;
non_zero_meas_dist(k.PSI) = 0.0001;
non_zero_meas_dist(k.V) = 0.03;
%% TODO specify other disturbances if required
model_add_dist = zeros(k.get_size(), 1); % model_dist_for_kst_final(k);

%% Where the generated primitives are stored
result_dir = ResultDir('out/primitives_for_edgar/v18');

%% TODO adapt if you have more CPUs available
num_workers = 1;

%% Reference trajectories of primitives to generate
primitive_duration = 1.0;
desired_duration = primitive_duration;
% The distance of the end positions of the generated primitives:
%% TODO change if needed. Smaller values means more primitives will be generated.
target_point_distance = 0.5;

% The accelerations of the generated primitives:
%% TODO change if needed
accs = {-2.0, 0.0, 1.5};

inp_creator = InpCreatorForADelta(k, i, params);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, false);

%% Initial sets that primitives are generated for
initial_sizes = dictionary();
initial_sizes(k.X) = 3.0 * non_zero_meas_dist(k.X);
initial_sizes(k.Y) = 3.0 * non_zero_meas_dist(k.Y);
initial_sizes(k.V) = 3.0 * non_zero_meas_dist(k.V);
initial_sizes(k.PSI) = 5.0 * non_zero_meas_dist(k.PSI);

interval_sampler = create_initial_sampler_for_edgar_inp_delta( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, initial_sizes);

%% Dynamics of closed-loop system
ref_state_builder = SameRefStateBuilder(k);
% control_params = [5000 * 0.0037, 7000 * 0.0104, 20000 * 0.0009, 19.7692, 8.3620];
control_params = [18.5, 72.8, 18, 19.7692, 8.3620];

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, ... % we integrate the measurement uncertainty in the state-processor
    true,  ...
    false, ... % we only us additive disturbances
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ...
    ref_state_builder, control_params);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, @pd_controller_inp_delta_4d, ...
    k, i, params, dyn_conf, state_proc);

create_dynamics = @() nonlinearSys('edgar_4d_inp_delta_v999', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size()); % , ...

%% Further settings
dt_controller = 0.01;

cora_options = create_cora_options_1();
cora_options.timeStep = dt_controller;
cora_options.maxError = 1e6 * ones(state_proc.get_state_size(), 1);

occ_comp = create_kst_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = false;
create_plot = true;
do_random_simulations = true;

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
    debug, create_plot, true, false, do_random_simulations, ...
    0, 0, cora_options);

sim = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);
sim_sampler = RandomSimSampler(state_meas_dist, model_add_dist, zeros(i.SIZE, 1));

%% TODO for testing purposes of the primitives themselves (without the real reachsets), the
%% following can be used:
% constant_set = interval(-5.0 * state_meas_dist, 5.0 * state_meas_dist);
% we reduce the final interval size as the normalization increase the
% size in some dimensions again
% final_constant_set = 0.1 * interval(-state_meas_dist, state_meas_dist);
% gen = ConstantReachPerformer(constant_set, final_constant_set);

% For real reachsets, use the following:
gen = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values);
set_gen = PrimitiveSetGenerator(settings, gen, params, ...
    inp_traj_gen, model_for_ref_traj, sim, sim_sampler, 50);

set_gen.start_generation(num_workers);

function sampler = create_initial_sampler_for_edgar_inp_delta( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, initial_sizes)
    k = state_order;

    sampling_dims = [k.V];
    other_dims = setdiff(1:k.get_size(), sampling_dims);

    sampling_bounds = dictionary();
    invert_sampling_order = dictionary();
    range_sizes = dictionary();

    sampling_bounds{k.V} = [params.v_min, params.v_max];
    invert_sampling_order{k.V} = 1;
    range_sizes{k.V} = 0.3;

    overlap_abs = dictionary(sampling_dims, 2 * initial_sizes(sampling_dims));

    other_dims_intervals = dictionary();
    for i = other_dims
        other_dims_intervals{i} = [-initial_sizes(i), initial_sizes(i)];
    end

    sampler = BoxLikeStateSpaceDivider(result_dir, ...
        num_workers, ...
        constraints, terminal_cond, k.get_size(), ...
        sampling_dims, NaN, sampling_bounds, ...
        range_sizes, ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
