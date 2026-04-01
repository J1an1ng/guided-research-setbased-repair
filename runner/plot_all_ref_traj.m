%% Model settings
model = @kst;
k = KSTStateOrder();
i = InpOrderADeltaVDefault;

result_dir = ResultDir('out/for_trajectory_sampling_truck_only_medium_lat_acc');
num_workers = 1;

%% Configs
params = create_kst_params_MAN();
state_meas_dist = meas_dist_for_kst_1(k);
terminal_cond = StandstillTerminalCondition(k, 1);
constraints = KSTConstraints(k, i, params);
interval_sampler = create_coarse_initial_sampler_for_truck_only( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);
[batch_idx, initial_sets] = interval_sampler.next_batch();

% x = [0; 0; -0.22; 21.11; 0; 0];
% initial_intervals = {interval(x, x)};

acc_max = -1.0;
accs = {-1.0, -0.5};

%% Traj generator
duration = 1.0;
desired_duration = duration;
target_point_distance = 0.1;
terminal_cond = StandstillTerminalCondition(k, 1);
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_state_bounds(k, params, 1);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, model, bounds, ...
    desired_duration, accs, target_point_distance, 0.01);
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    duration, 0.01, 2, model, k, i, params, constraints);

%% Occ Comp
occ_comp = create_kst_occ_comp(k, params);
truck_plotter = KSTPlotter(occ_comp, k, params);

fprintf("Num interval: %d \n", length(initial_sets));

out_dir = "out/trajectory_sampling_truck_only_medium_lat_acc";
mkdir(out_dir);

plot_discarded = 1;

for i = 1:length(initial_sets)
    init_set = initial_sets{i};

    plot_trajectories(init_set.cora_set, truck_plotter, valid_inp_traj_gen, ...
        k, plot_discarded, out_dir, i);
end
