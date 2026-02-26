%% -----------------------------------------------------------------------------------------------------
%% Model settings
%% -----------------------------------------------------------------------------------------------------
model = @kst_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
model_for_ref_traj = prevent_backward_driving(model);

%% -----------------------------------------------------------------------------------------------------
%% Configs
%% -----------------------------------------------------------------------------------------------------
veh_params = create_kst_trailer_params_MAN_v2();
state_meas_dist = meas_dist_for_kst_trailer_final(k); % zeros(k.get_size(), 1);
model_add_dist = model_dist_for_kst_trailer_final(k); % zeros(k.get_size(), 1);

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
duration = 2.0;
time_decimal_places = 2;

constraints = KSTTrailerConstraints(k, i, veh_params);

result_dir = ResultDir('out/simulation_pp_no_obs_final');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
control_params = [0.1, 12.8747, 2.5546];
% control_params = [0.5, 1.0, 10.0];
% 55.7358, 79.7620

ref_state_args = dictionary();
pure_pursuit_lookahead_time = 0.5;
ref_state_args("pure_pursuit_lookahead_num_steps") = double(uint32(pure_pursuit_lookahead_time / dt_controller));

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));

ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ref_state_builder, control_params, ref_state_args);

inp_bounds = create_inp_bounds_for_a_deltav(i, veh_params);
check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

dist_proc = KSTDistProcForRotatedPositionalDists(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, ~] = create_cont_time_discr_contr_model( ...
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k), ...
    k, i, veh_params, dyn_conf, state_proc); % , check_inp);

create_dynamics = @() nonlinearSys('my_final_model', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

simulator = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);

%% -----------------------------------------------------------------------------------------------------
%% Generation of reference trajectories
%% -----------------------------------------------------------------------------------------------------

%% Coarse initial sets covering roughly the entire state space
initial_sizes = dictionary();
initial_sizes(k.X) = 1 * state_meas_dist(k.X);
initial_sizes(k.Y) = 0.8 * state_meas_dist(k.Y);
initial_sizes(k.DELTA) = 2 * state_meas_dist(k.DELTA);
initial_sizes(k.V) = 2 * state_meas_dist(k.V);
initial_sizes(k.PSI) = 2 * state_meas_dist(k.PSI);
initial_sizes(k.HA) = 2 * state_meas_dist(k.HA);

%% Initial state
v = 15.0;
initial_state = zeros(k.get_size(), 1);
initial_state(k.V) = v;

%% Reference trajectory
a = -2.0;
pp_time_constant = 0.05;

inp_creator = InpCreatorForADeltaV(k, i, veh_params, pp_time_constant);

total_duration = abs(v / a);
total_dist = abs((v^2) / (2 * a));

lane_width = 3.75;
shoulder_width = 2.75;
evasive_dist = lane_width / 2.0 + shoulder_width / 2.0;

a_fun = create_acc_function(a, total_duration / 2);

dist_1 = total_dist / 2.0;
duration_1 = (-v + sqrt(v^2 + 2 * a * dist_1)) / a;
duration_1 = 0.9 * duration_1; % for preventing "overshooting" in pure pursuit

traj_part_1 = KSTInpTrajTargetPoint(k, veh_params, constraints, ...
    a, a_fun, [dist_1; -evasive_dist / 2.0], inp_creator);

offset_2 = 10.0;
traj_part_2 = KSTInpTrajTargetPoint(k, veh_params, constraints, ...
    a, a_fun, [total_dist + offset_2; -evasive_dist], inp_creator);
traj = CombinedTraj({traj_part_1; traj_part_2}, [duration_1; total_duration]);

ref_traj_sim = Simulator(terminal_cond, time_decimal_places, ...
    model_for_ref_traj, k, i, veh_params, constraints, ...
    pure_pursuit_lookahead_time, @process_final_state_for_ext_horizon);
success = ref_traj_sim.simulate(traj, initial_state, total_duration, dt_controller);

if ~success
    warning("Invalid trajectory");
end

%% -----------------------------------------------------------------------------------------------------
%% Optimization
%% -----------------------------------------------------------------------------------------------------
initial_itv_rads = zeros(k.get_size(), 1);
for ii = 1:k.get_size()
    initial_itv_rads(ii, 1) = initial_sizes(ii);
end

sim_options.points = 10;
sim_options.fracVert = 0.5;
sim_options.fracInpVert = 0.5;
sim_options.nrConstInp = 1;
% Important: use a recent enough version of CORA that supports simResult.error

errors_to_catch = [InputOutOfBoundsException.ID];
num_sims = 5;
sim_sampler = LateralWCSimSampler(k, i, state_meas_dist, model_add_dist, []);
sim_res = simulator.simulate(num_sims, traj, initial_itv_rads, dt_controller, errors_to_catch, sim_sampler, dyn_conf_values);
% sim_res_x_cell = arrayfun(@(r) cell2mat(r.x), sim_res, 'UniformOutput', false);
% sim_res_x = cat(3, sim_res_x{:});

figure;

occ_comp = create_kst_trailer_occ_comp(k, veh_params);
truck_plotter = KSTPlotter(occ_comp, k, veh_params);
truck_plotter.plot_vehicle(traj.states(:, 1), 'r');
hold on;

plot(traj.states(1, :), traj.states(2, :));
hold on;
for i = 1:length(sim_res)
    sim_res_x = cell2mat(sim_res(i).x);
    plot(sim_res_x(:, 1), sim_res_x(:, 2));
end
xlabel('m', 'FontSize', 18);
ylabel('m', 'FontSize', 18);
set(get(gca, 'ylabel'), 'rotation', 0);
axis equal;
ylim([-7, 4]);
