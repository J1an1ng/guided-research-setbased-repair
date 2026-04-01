%% Model settings
model = @kst_trailer_inp_delta;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
mirroring = KSTTrailerInpDeltaMirroring(k);
norm_final_set = @norm_kst;
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN_v2();

%% FIXME update to finally used uncertainties
state_meas_dist = meas_dist_for_kst_trailer_inp_delta_final(k);
state_meas_dist_set = zonotope(interval(-state_meas_dist, state_meas_dist));
% model_add_dist = zeros(k.SIZE, 1); % model_dist_for_kst_trailer_final(k);

model_add_dist_y_and_psi = 0.5 * [state_meas_dist(k.Y); state_meas_dist(k.PSI)] .* zonotope([0; 0], [1 1 0; -1 0 1]);
model_add_dist = cartProd(project(state_meas_dist_set, [k.X k.V k.HA]), model_add_dist_y_and_psi);
model_add_dist = project(model_add_dist, [1 4 2 5 3]);

terminal_cond = StandstillTerminalCondition(k, 1);
time_decimal_places = 2;
constraints = KSTTrailerInpDeltaConstraints(k, i, params);

pure_pursuit_lookahead_time = 0.5;
dt_controller = 0.01;

reduced_k = ReducedKSTStateOrder();
position_k = PositionStateOrder();

%% Set up dynamics
ref_state_builder = LookaheadRefStateBuilder(k, ...
    reduced_k.create_state_order_mapping(k), position_k.create_state_order_mapping(k));
control_params = [19.7692, 16.7240];

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
    model, create_lat_pure_pursuit_long_pd(reduced_k, position_k, @create_inp_for_a_delta), ...
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
    pure_pursuit_lookahead_time, @(x, ~) x);

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
y_0(k.V) = 15.0;
y_0(k.PSI) = 0.0;
y_0(k.HA) = 0.0;

delta = 0.5 * constraints.delta_max_for_v(y_0(k.V));

u_ref = zeros(i.SIZE, 2);
u_ref(i.DELTA, 1) = delta;
u_ref(i.A, 1) = 1.0;
u_ref(i.DELTA, 2) = delta;
u_ref(i.A, 2) = 1.0;

[R_final, occs] = online_verifier.compute_reachset(y_0, u_ref, 0.2, true);

figure;
