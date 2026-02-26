%% -----------------------------------------------------------------------------------------------------
%% Model settings
%% -----------------------------------------------------------------------------------------------------
model = @point_mass_long;
k = PointMassLongStateOrder();
inp_order = InpOrderA();
model_for_ref_traj = prevent_backward_driving(model);

%% -----------------------------------------------------------------------------------------------------
%% Configs
%% -----------------------------------------------------------------------------------------------------
veh_params = create_kst_trailer_params_MAN_v2();
state_meas_dist = meas_dist_for_point_mass_long_final(k); % zeros(k.get_size(), 1);
model_add_dist = model_dist_for_point_mass_long_final(k); % zeros(k.get_size(), 1);

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);

duration = 1.0;
desired_duration = duration;
time_decimal_places = 2;

constraints = PointMassLongConstraints(k, inp_order, veh_params);

result_dir = ResultDir('out/reach_long');
check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");

%% -----------------------------------------------------------------------------------------------------
%% Dynamics
%% -----------------------------------------------------------------------------------------------------
control_params = [19.7692, 8.3620];
ref_state_builder = SameRefStateBuilder(k);

dyn_conf = ControlledModelParamConfig(false, ...
    k, inp_order, ...
    false, true, false, ...
    ref_state_builder.get_ref_state_size(), true, length(control_params));

dyn_conf_values = ControlledModelParamValues( ...
    [], model_add_dist, [], ...
    [], ref_state_builder, control_params);

dist_proc = PointMassLongDistProc(k);
state_proc = StateProcessorForDiscrContrWithDistProc(k, state_meas_dist, dist_proc);

[system_dynamics, ~] = create_cont_time_discr_contr_model( ...
    model, @pd_controller_long_only, ...
    k, inp_order, veh_params, dyn_conf, state_proc);

create_dynamics = @() nonlinearSys('my_final_model_long_only', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size());

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

%% TODO choose traj:
traj = trajs(1);

initial_itv_rad = zeros(k.get_size(), 1);
% Hint: we use larger itvs to make controller optimization more robust
initial_itv_rad(k.S, 1) = 10 * state_meas_dist(k.S);
initial_itv_rad(k.V, 1) = 1.5;

initial_itv = interval(traj.states(:, 1) - initial_itv_rad, traj.states(:, 1) + initial_itv_rad);
initial_set = SetContainer(1, initial_itv, 1);

num_workers = 1;
debug = false;
create_plot = true;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, inp_order, veh_params, ...
    [], ...
    constraints, ...
    [], ...
    [], ...
    state_meas_dist, [], model_add_dist, ...
    dt_controller, 2, 0.1, ...
    terminal_cond, ...
    duration, ...
    false, false, ...
    [], ...
    [], ...
    debug, create_plot, true, false, do_random_simulations);

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");

p = Primitive(initial_set, traj, settings.dt_control, settings.occ_dt);

reach_performer = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values, []);

dyn = create_dynamics();

log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
log_fid = fopen(log_file_path, 'wt');

res = reach_performer.generate(settings, p, dyn, log_fid);
