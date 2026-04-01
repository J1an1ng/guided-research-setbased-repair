%% System dynamics
model = @kst_trailer_inp_delta;
dt_controller = 0.01;
time_decimal_places = 2;
k = KSTTrailerInpDeltaStateOrder();
i = InpOrderADelta();
veh_params = create_kst_trailer_params_MAN();

%% FIXME: these are the actual initial guesses:
state_meas_dist_non_zero = meas_dist_for_kst_trailer_inp_delta_1(k);
% model_add_dist = discr_time_kst_trailer_inp_delta_add_dist_1(k);
%% ...but they result in large reachsets, thus we take the zero dists
state_meas_dist = zero_meas_dist_for_kst_trailer_inp_delta(k);
model_add_dist = zero_kst_trailer_inp_delta_add_dist(k);

control_params = [0.036, 0.72, 55.7358, 79.7620];

ref_state_builder = SameRefStateBuilder(k);

dyn_conf = ControlledModelParamConfig(k, i, ...
    state_meas_dist, true, ...
    model_add_dist, true, ...
    [], false, ...
    ref_state_builder, true, control_params, length(control_params));

[system_dynamics, system_output] = create_time_continuous_controlld_model();
[system_dynamics_wrapper, system_output_wrapper] = build_controlled_model( ...
    system_dynamics, system_output, model, ...
    @pd_controller_abx, ...
    k, i, veh_params, dyn_conf);

dyn = nonlinearSys('cont_time_kst_trailer_with_inp_delta', ...
    system_dynamics_wrapper, ...
    k.get_size(), dyn_conf.get_params_size(), ...
    system_output_wrapper, k.get_size());

%% Create reference trajectory - to be replaced with the one used during the conformance checking test drive
desired_duration = 1.0;
accs = {-0.5, -1.0, -1.5};
target_point_distance = 0.1;

initial_state = zeros(k.get_size(), 1);
% States that can be transformed to 0:
initial_state(k.X) = 0.0;
initial_state(k.Y) = 0.0;
initial_state(k.PSI) = 0.0;
% Absolute states:
initial_state(k.V) = veh_params.v_max - 1.0; % we stay below the max velocity because of measurement uncertainty
initial_state(k.HA) = 0.0;

constraints = KSTTrailerInpDeltaConstraints(k, i, veh_params);
terminal_cond = StandstillTerminalCondition(k, 1);
inp_creator = InpCreatorForADelta(k, i, veh_params);
inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, veh_params, ...
    constraints, inp_creator, ...
    model, desired_duration, accs, target_point_distance, true);
valid_inp_traj_gen = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
    desired_duration, dt_controller, time_decimal_places, model, k, i, veh_params, constraints);
[trajectories, ~] = valid_inp_traj_gen.generate(initial_state);
ref_traj = trajectories{1}; %% any index is fine

%% Reach params
[params.u, params.U] = dyn_conf.create_params(ref_traj, 1:size(ref_traj.inputs, 2));

params.tStart = 0;
params.tFinal = ref_traj.duration;
params.R0 = zonotope(interval(initial_state - state_meas_dist_non_zero, initial_state + state_meas_dist_non_zero));

options = create_cora_options_1();

reachsets = reach(dyn, params, options);

%% Plot result
f = figure;
cols = 3;
tiledlayout(cols, ceil(k.get_size() / cols));

for i = 1:k.get_size()
    nexttile;
    if ~isempty(reachsets)
        plotOverTime(reachsets, i, 'LineWidth', 3.0);
    end

    hold on;
    plot(ref_traj.times, ref_traj.states(i, :), 'Color', 'yellow');

    title(sprintf('%d', i));
end
