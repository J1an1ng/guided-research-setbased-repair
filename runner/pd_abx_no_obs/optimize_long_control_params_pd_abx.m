%% -----------------------------------------------------------------------------------------------------
%% Model settings
%% -----------------------------------------------------------------------------------------------------
model = @point_mass_long;
k = PointMassLongStateOrder;
i = InpOrderA;
model_for_ref_traj = prevent_backward_driving(model);

%% -----------------------------------------------------------------------------------------------------
%% Configs
%% -----------------------------------------------------------------------------------------------------
veh_params = create_kst_trailer_params_MAN();
state_meas_dist = meas_dist_for_point_mass_long_1(k);
model_add_dist = discr_time_point_mass_long_add_dist_1(k);
dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
duration = 1.0;
time_decimal_places = 2;

constraints = PointMassLongConstraints(k, i, veh_params);

result_dir = ResultDir('out/opt_long_control_pd_abx_no_obs_2');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
inp_bounds = create_inp_bounds_for_a(i, veh_params);
control_params_lb = [0., 0.];
control_params_ub = [60., 80.];

ref_state_builder = SameRefStateBuilder(k);

dyn_conf = ControlledModelParamConfig(true, ...
    k, i, ...
    true, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params_lb));

dyn_conf_values = ControlledModelParamValues( ...
    state_meas_dist, model_add_dist, [], ...
    ref_state_builder, []);

check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

[system_dynamics, system_output] = create_time_discrete_controlled_model(dt_controller);
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    @pd_controller_abx_long_only, ...
    k, i, veh_params, dyn_conf, check_inp);

dyn = nonlinearSysDT('discr_time_point_mass_long_with_inp_a', ...
    system_dynamics_wrapper, dt_controller, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, k.get_size());

%% -----------------------------------------------------------------------------------------------------
%% Generation of reference trajectories
%% -----------------------------------------------------------------------------------------------------
accs = {-0.5, -1.0, -2.0};
inp_traj_gen = PointMassLongInpTrajGenerator(k, i, accs);
max_speed = veh_params.v_max;
speed_step = 5.0;
speed_num = floor(max_speed / speed_step);

initial_itv_rad = zeros(k.get_size(), 1);
initial_itv_rad(k.S, 1) = state_meas_dist(k.S);
% Hint this is larger than we used to use for sampling to make controller optimization more robust
initial_itv_rad(k.V, 1) = 0.5;

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
        model_for_ref_traj, k, i, veh_params, ...
        constraints);

    [trajectories, ~] = traj_gen.generate(initial_state);
    trajs(counter:counter + length(trajectories) - 1) = trajectories(:);
    counter = counter + length(trajectories);
end

trajs = [trajs{:}];

%% -----------------------------------------------------------------------------------------------------
%% Optimization
%% -----------------------------------------------------------------------------------------------------
cost_def = SimRefPathDeviationCost([k.S], [1.0]);
opt = SimBasedControlWeightOptimizer(trajs, dyn, dyn_conf, dyn_conf_values, ...
    100, initial_itv_rad, control_params_lb, control_params_ub, check_point_file_path, cost_def, true);
opt_ctrl_params = opt.optimize();
display(opt_ctrl_params);
%% -----------------------------------------------------------------------------------------------------

%% with ref acceleration added to control input: 7.6989    2.4944
%% without: 4.8928    2.4071
