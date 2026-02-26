%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
mirroring = KSTTrailerMirroring(k);
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

%% FIXME update to finally used uncertainties
state_meas_dist = meas_dist_for_kst_trailer_final(k);
state_meas_dist_set = zonotope(interval(-state_meas_dist, state_meas_dist));
model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_trailer_final(k);

terminal_cond = StandstillTerminalCondition(k, 1);
time_decimal_places = 2;
constraints = KSTTrailerConstraints(k, i, params);

pure_pursuit_lookahead_time = 0.3;
dt_controller = 0.01;

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

%% Set up dynamics
ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [0.1, 19.7692, 8.3620];

ref_state_args = dictionary();
ref_state_args("pure_pursuit_lookahead_num_steps") = double(uint32(pure_pursuit_lookahead_time / dt_controller));

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
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k), ...
    k, i, params, dyn_conf, state_proc);

create_dynamics = @() nonlinearSys('my_final_model', ...
    system_dynamics, ...
    state_proc.get_state_size(), dyn_conf.get_params_size()); % , ...
% system_output, k.get_size());

dyn = create_dynamics();

r_init_computer = KSTRInitComputerForRotatedPositionalDists(k);
cora_options = create_cora_options_1();
sim = Simulator(terminal_cond, time_decimal_places, ...
    model_for_ref_traj, k, i, params, constraints, ...
    pure_pursuit_lookahead_time, @process_final_state_for_ext_horizon);

occ_comp = create_kst_trailer_occ_comp(k, params);
occ_dt = 0.1;
plot_result = true;

online_verifier = OnlineVerifier( ...
    dyn, k, r_init_computer, state_meas_dist_set, cora_options, ...
    state_proc, dyn_conf, dyn_conf_values, sim, occ_comp, norm_final_set, mirroring, ...
    dt_controller, occ_dt, plot_result);

y_0 = zeros(k.get_size(), 1);
y_0(k.X) = 0.0;
y_0(k.Y) = 0.0;
y_0(k.DELTA) = 0.0;
y_0(k.V) = 15.0;
y_0(k.PSI) = 0.0;
y_0(k.HA) = 0.0;

u_ref = zeros(i.SIZE, 2);
u_ref(i.DELTA_V, 1) = 0.0;
u_ref(i.A, 1) = 1.0;
u_ref(i.DELTA_V, 2) = 0.0;
u_ref(i.A, 2) = 1.0;

online_verifier.compute_reachset(y_0, u_ref, 0.2);
