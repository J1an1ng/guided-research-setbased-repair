%% -----------------------------------------------------------------------------------------------------
%% Model settings
%% -----------------------------------------------------------------------------------------------------
model = @point_mass_long;
k = PointMassLongStateOrder;
inp_order = InpOrderA;
model_for_ref_traj = prevent_backward_driving(model);

%% -----------------------------------------------------------------------------------------------------
%% Configs
%% -----------------------------------------------------------------------------------------------------
veh_params = create_kst_trailer_params_MAN_v2();
state_meas_dist = meas_dist_for_point_mass_long_final(k); % zeros(k.get_size(), 1);
model_add_dist = model_dist_for_point_mass_long_final(k); % zeros(k.get_size(), 1);

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
duration = 2.0;
time_decimal_places = 2;

constraints = PointMassLongConstraints(k, inp_order, veh_params);

result_dir = ResultDir('out/sim_long');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
control_params = [19.7692, 8.3620];
% control_params = [0.5, 1.0, 10.0];
% 55.7358, 79.7620

ref_state_builder = SameRefStateBuilder(k);

dyn_conf = ControlledModelParamConfig(false, ...
    k, inp_order, ...
    false, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ref_state_builder, control_params);

inp_bounds = create_inp_bounds_for_a(inp_order, veh_params);
check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

dist_proc = PointMassLongDistProc(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, ~] = create_cont_time_discr_contr_model( ...
    model, @pd_controller_long_only, ...
    k, inp_order, veh_params, dyn_conf, state_proc, check_inp);

create_dynamics = @() nonlinearSys('my_final_model_long_only', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

simulator = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);

% sim_sampler = RandomSimSampler(state_meas_dist, model_add_dist, []);
sim_sampler = LongWCSimSampler(k.S, k.V, inp_order.A, state_meas_dist, model_add_dist, []);

%% -----------------------------------------------------------------------------------------------------
%% Generation of reference trajectories
%% -----------------------------------------------------------------------------------------------------

accs = {-0.5, -1.5, -2.5};
inp_traj_gen = PointMassLongInpTrajGenerator(k, inp_order, accs);
max_speed = veh_params.v_max;
speed_step = 5.0;
speed_num = floor(max_speed / speed_step);

trajs = cell(speed_num * length(accs), 1);
counter = 1;

for i_speed = 1:speed_num
    speed = max_speed - (i_speed - 1) * speed_step;
    initial_state = zeros(k.get_size(), 1);
    initial_state(k.S, 1) = 0.0;
    initial_state(k.V, 1) = speed;

    traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
        duration, ...
        dt_controller, ...
        time_decimal_places, ...
        model_for_ref_traj, k, inp_order, veh_params, ...
        constraints);

    [trajectories, ~] = traj_gen.generate(initial_state);
    trajs(counter:counter + length(trajectories) - 1) = trajectories(:);
    counter = counter + length(trajectories);
end

trajs = [trajs{:}];

%% -----------------------------------------------------------------------------------------------------
%% Optimization
%% -----------------------------------------------------------------------------------------------------
initial_itv_rad = zeros(k.get_size(), 1);
% Hint: we use larger itvs to make controller optimization more robust
initial_itv_rad(k.S, 1) = 3 * state_meas_dist(k.S);
initial_itv_rad(k.V, 1) = 0.5;

sim_options.points = 10;
sim_options.fracVert = 0.5;
sim_options.fracInpVert = 0.5;
sim_options.nrConstInp = 1;
% Important: use a recent enough version of CORA that supports simResult.error

errors_to_catch = [InputOutOfBoundsException.ID];
num_sims = 10;
traj = trajs(8);
sim_res = simulator.simulate(num_sims, traj, initial_itv_rad, dt_controller, errors_to_catch, sim_sampler, dyn_conf_values);
% sim_res_x_cell = arrayfun(@(r) cell2mat(r.x), sim_res, 'UniformOutput', false);
% sim_res_x = cat(3, sim_res_x{:});

%% Plot distance
figure;

plot(traj.times, traj.states(k.S, :), 'k', 'LineWidth', 2);
hold on;
for i = 1:length(sim_res)
    sim_res_x = cell2mat(sim_res(i).x);
    sim_res_t = cell2mat(sim_res(i).t);
    plot(sim_res_t, sim_res_x(:, k.S));
end
xlabel('s', 'FontSize', 18);
ylabel('m', 'FontSize', 18);
set(get(gca, 'ylabel'), 'rotation', 0);
title('Distance');
% axis equal;

%% Plot speed
figure;
plot(traj.times, traj.states(k.V, :), 'k', 'LineWidth', 2);
hold on;
for i = 1:length(sim_res)
    sim_res_x = cell2mat(sim_res(i).x);
    sim_res_t = cell2mat(sim_res(i).t);
    plot(sim_res_t, sim_res_x(:, k.V));
end
xlabel('s', 'FontSize', 18);
ylabel('m/s', 'FontSize', 18);
set(get(gca, 'ylabel'), 'rotation', 0);
title('Speed');

%% Plot acceleration
figure;
plot(traj.times(2:end), traj.inputs(inp_order.A, :), 'k', 'LineWidth', 2);
hold on;
for i = 1:length(sim_res)
    sim_res_x = cell2mat(sim_res(i).x);
    sim_res_t = cell2mat(sim_res(i).t);
    accs = diff(sim_res_x(:, k.V)) ./ diff(sim_res_t);
    plot(sim_res_t(2:end), accs);
end
xlabel('s', 'FontSize', 18);
ylabel('m/s^2', 'FontSize', 18);
set(get(gca, 'ylabel'), 'rotation', 0);
title('Acceleration');
