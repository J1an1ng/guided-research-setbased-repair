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
veh_params = create_kst_trailer_params_MAN_v2();
state_meas_dist = meas_dist_for_point_mass_long_final(k);
model_add_dist = model_dist_for_point_mass_long_final(k);

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
duration = 1.0;
time_decimal_places = 2;

constraints = PointMassLongConstraints(k, i, veh_params);

result_dir = ResultDir('out/opt_long_control_pp_no_obs_final_2');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
inp_bounds = create_inp_bounds_for_a(i, veh_params);
control_params_lb = [0., 0.];
control_params_ub = [20.0, 30.0];

ref_state_builder = SameRefStateBuilder(k);

dyn_conf = ControlledModelParamConfig(false, ...
    k, i, ...
    false, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params_lb));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ref_state_builder, []);

check_inp = @(x, u) inp_bounds.get_relative_bound_violations(u);

dist_proc = PointMassLongDistProc(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, ~] = create_cont_time_discr_contr_model( ...
    model, @pd_controller_long_only, ...
    k, i, veh_params, dyn_conf, state_proc, check_inp);

create_dynamics = @() nonlinearSys('my_final_model_long_only', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

simulator = RandomCoraSimulator(create_dynamics(), dyn_conf, dyn_conf_values, state_proc);
sim_sampler = RandomSimSampler(state_meas_dist, model_add_dist, []);

%% -----------------------------------------------------------------------------------------------------
%% Generation of reference trajectories
%% -----------------------------------------------------------------------------------------------------
accs = {-0.5, -1.5, -2.5};
inp_traj_gen = PointMassLongInpTrajGenerator(k, i, accs);
max_speed = veh_params.v_max;
speed_step = 5.0;
speed_num = floor(max_speed / speed_step);

initial_itv_rad = zeros(k.get_size(), 1);
% Hint: we use larger itvs to make controller optimization more robust
initial_itv_rad(k.S, 1) = 3 * state_meas_dist(k.S);
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
empty_dict = configureDictionary("string", "double");
cost_def = SimRefPathDeviationCost([k.S], [1.0]);
opt = SimBasedControlWeightOptimizer(trajs, simulator, sim_sampler, dt_controller, ...
    10, initial_itv_rad, control_params_lb, control_params_ub, ...
    empty_dict, empty_dict, ...
    check_point_file_path, cost_def, false);
opt_ctrl_params = opt.optimize();
display(opt_ctrl_params);
%% -----------------------------------------------------------------------------------------------------

%% Result:
%% 12.8747    2.5546

%% MaxFunctionEvaluations: 1000
%% 19.7692    8.3620
