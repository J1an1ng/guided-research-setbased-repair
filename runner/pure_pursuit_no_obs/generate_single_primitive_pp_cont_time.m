%% Model settings
model = @kst_trailer_inp_delta;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
mirroring = KSTTrailerInpDeltaMirroring(k);
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

state_meas_dist = meas_dist_for_kst_trailer_inp_delta_final(k);

% model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_trailer_inp_delta_final(k);

state_meas_dist_set = zonotope(interval(-state_meas_dist, state_meas_dist));
% model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_trailer_final(k);

model_add_dist_y_and_psi = 0.5 * [state_meas_dist(k.Y); state_meas_dist(k.PSI)] .* zonotope([0; 0], [1 1 0; -1 0 1]);
model_add_dist = cartProd(project(state_meas_dist_set, [k.X k.V k.HA]), model_add_dist_y_and_psi);
model_add_dist = project(model_add_dist, [1 4 2 5 3]);

model_add_dist_scale = 0.88;

% too large: 1.3, 1.6, 2.0, 2.5, 3.0
model_add_dist = model_add_dist_scale * model_add_dist;

%% FIXME we specify the initial sizes based on the measurement uncertainties for now,
%% but this needs to be changed based on the reachability analysis for the nominal input
initial_rads = dictionary();
initial_rads(k.X) = 0.1; % 2 * state_meas_dist(k.X);
initial_rads(k.Y) = 0.15; % 4.0 * state_meas_dist(k.Y);
initial_rads(k.V) = 0.01; % 0.2; % 4 * state_meas_dist(k.V);
initial_rads(k.PSI) = 0.03; % 0.019; % 5 * state_meas_dist(k.PSI); % 2*
initial_rads(k.HA) = 0.01; % 0.1 %3 * state_meas_dist(k.HA);

constraints = KSTTrailerInpDeltaConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/pure_pursuit_no_obs/single_primitive_inp_delta_2');

primitive_duration = 1.0;
desired_duration = primitive_duration;
target_point_distance = 0.1;

accs = {-0.5, -1.0, -1.5};

inp_creator = InpCreatorForADelta(k, i, params);
% inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
%     constraints, inp_creator, ...
%    model, desired_duration, accs, target_point_distance, false);

lookahead_distance = 10.0; % 9.0; %18.0;
% 20.0: too large for low speed (y grows)
% 5.0: too small for low speed (ha grows and occ is too broad)
% 7.0: worse than 9.0
% 15.0: works but worse than 9.0
% 9.0 - 11.0: work well

inp_traj_gen = KSTInpTrajTargetLineGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, primitive_duration, accs, target_point_distance, ...
    [], lookahead_distance);

% pure_pursuit_lookahead_time = 2.0; % 0.5;
dt_controller = 0.01;

% pure_pursuit_lookahead_offset = 2.0; % 0.5;
extended_horizon = 0.0;
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, dt_controller, 2, model, k, i, params, constraints, ...
        extended_horizon, @(x, ~) x);

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

%% Set up dynamics
ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [19.7692, 30]; % 16.7240]; %8.3620 %40.7240

ref_state_args = dictionary();
% ref_state_args("pure_pursuit_lookahead_num_steps") = double(uint32(pure_pursuit_lookahead_time / dt_controller));
% Hint: this does not have to be the same as the lookahead distance for the ref traj:
ref_state_args("lookahead_distance") = lookahead_distance;

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, ... % we integrate the measurement uncertainty in the state-processor
    true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ...
    ref_state_builder, control_params, ref_state_args);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k, @create_inp_for_a_delta), ...
    k, i, params, dyn_conf, state_proc);

create_dynamics = @() nonlinearSys('my_final_model_inp_delta_2', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

num_workers = 1;
interval_sampler = create_base_initial_sampler_for_hierarchical_for_inp_delta( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, initial_rads);

occ_comp = create_kst_trailer_occ_comp(k, params);
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
    extended_horizon, @(x, ~) x);

[batch_idx, initial_sets] = interval_sampler.next_batch();

%% FIXME
%% initial_set = initial_sets{1};
% initial_set = initial_sets{3000};
% initial_set = initial_sets{36};
% initial_set = initial_sets{1};
% initial_set = initial_sets{2};
% initial_set = initial_sets{100};
% initial_set = initial_sets{180};
% initial_set = initial_sets{250};
initial_set = initial_sets{50};

%% FIXME
psi_and_ha = 0.5 * [0.03; 0.035] .* zonotope([0; 0], [1 1 0; -1 0 1]);
ss = cartProd(project(initial_set.cora_set, [1 2 3]), psi_and_ha);
initial_set.cora_set = ss; % project(ss, [1 4 2 5 3]);

display(initial_set.cora_set);

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, valid_inp_traj_gen);

reach_performer = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values);

%% FIXME
%% p = p{1};
% p = p{9};
% p = p{28};
% p = p{3};
% p = p{7};
% p = p{8};
p = p{47};

dyn = create_dynamics();

log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
log_fid = fopen(log_file_path, 'wt');

res = reach_performer.generate(settings, p, dyn, log_fid);

sim_res = [];
if settings.create_plot && settings.do_random_simulations
    sim = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);
    sim_sampler = WorstCaseSimSamplerForKST(k, i, state_meas_dist_set, model_add_dist, [], [k.Y, k.PSI k.HA], [k.X, k.V], [i.DELTA], [i.A]);

    % initial_itv_rads = zeros(k.get_size(), 1);
    % initial_itv_rads(1:k.get_size()) = initial_rads(1:k.get_size());
    % initial_ref_state = ref_traj.states(:, 1);
    % initial_set = zonotope(interval(initial_ref_state - initial_itv_rads, initial_ref_state + initial_itv_rads));

    sim_res = sim.simulate(100, p.ref_traj, p.start_set.cora_set, dt_controller, [], sim_sampler);
end

if settings.create_plot
    plot_primitive(settings, p, res.reachsets, sim_res);
    plot_occupancy_and_save(settings, p, res.reachsets, 10);
end

rads_final_set = rad(interval(res.reachsets(end).timePoint.set{end}));
display(rad(interval(initial_set.cora_set)));
display(rads_final_set);

create_occupancy_plot(occ_comp, k, params, initial_set.c, p.ref_traj, res, max_half_width, 10);

ylim([-3, 3]);
