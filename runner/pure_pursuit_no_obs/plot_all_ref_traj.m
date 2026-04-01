%% Model settings
model = @kst_trailer_inp_delta;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();

result_dir = ResultDir('out/pure_pursuit_no_obs/all_ref_traj');
num_workers = 1;

%% Configs
params = create_kst_trailer_params_MAN();
state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k);

constraints = KSTTrailerInpDeltaConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

interval_sampler = create_coarse_initial_sampler_for_kst_trailer_inp_delta( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

[batch_idx, initial_sets] = interval_sampler.next_batch();

accs = {-0.5, -1.0, -1.5};

primitive_duration = 1.0;
desired_duration = primitive_duration;
target_point_distance = 0.1;

inp_creator = InpCreatorForADelta(k, i, params);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, true);

pure_pursuit_lookahead_time = 0.2;

valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, 0.01, 2, model, k, i, params, constraints, pure_pursuit_lookahead_time, @(x, ~) x);

%% Occ Comp
occ_comp = create_kst_trailer_occ_comp(k, params);
truck_plotter = KSTPlotter(occ_comp, k, params);

fprintf("Num interval: %d \n", length(initial_sets));

out_dir = result_dir.ref_traj_out_path;

plot_discarded = 1;

for i = 1:length(initial_sets)
    init_set = initial_sets{i};

    plot_trajectories(init_set.cora_set, truck_plotter, valid_inp_traj_gen, ...
        k, plot_discarded, out_dir, i);
end
