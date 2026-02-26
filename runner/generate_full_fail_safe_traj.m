model = @kst_trailer;
model_for_ref_traj = prevent_backward_driving(model);
k = KSTTrailerStateOrder();
i = InpOrderADeltaVDefault;
params = create_kst_trailer_params_2();
constraints = KSTTrailerConstraints(k, i, params);

state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);

f = create_get_delta_rad_for_v(constraints, state_meas_dist(k.DELTA));

v = 3.0; %% FIXME 15.0
delta_size = f(v);
start_itv = zeros(k.get_size(), 2);
start_itv(k.X, :) = [-state_meas_dist(k.X), state_meas_dist(k.X)];
start_itv(k.Y, :) = [-state_meas_dist(k.Y), state_meas_dist(k.Y)];
start_itv(k.DELTA, :) = [-delta_size, delta_size];
start_itv(k.V, :) = [v - state_meas_dist(k.V), v + state_meas_dist(k.V)];
start_itv(k.PSI, :) = [-state_meas_dist(k.PSI), state_meas_dist(k.PSI)];
start_itv(k.HA, :) = [-state_meas_dist(k.HA), state_meas_dist(k.HA)];

start_set = SetContainer(1, interval(start_itv(:, 1), start_itv(:, 2)), true);

pure_pursuit_time_constant = 0.01;

a = -2.0;
total_duration = abs(v / a);
total_dist = abs((v^2) / (2 * a));

lane_width = 3.75;
shoulder_width = 2.75;
evasive_dist = lane_width / 2.0 + shoulder_width / 2.0;

a_fun = create_acc_function(a, total_duration / 2);

dist_1 = total_dist / 2.0;
duration_1 = (-v + sqrt(v^2 + 2 * a * dist_1)) / a;
duration_1 = 0.9 * duration_1; % for preventing "overshooting" in pure pursuit

traj_part_1 = KSTInpTrajTargetPoint(k, i, ...
    params, a, a_fun, [dist_1; -evasive_dist / 2.0], pure_pursuit_time_constant);

offset_2 = 10.0;
traj_part_2 = KSTInpTrajTargetPoint(k, i, ...
    params, a, a_fun, [total_dist + offset_2; -evasive_dist], pure_pursuit_time_constant);
traj = CombinedTraj({traj_part_1; traj_part_2}, [duration_1; total_duration]);

result_dir = ResultDir('out/full_fail_safe_traj');
ref_state_builder = SameRefStateBuilder(k);
[weights_size, f] = create_pd_control_weights_fun_with_increase_speed_wise(k);
dyn_conf = ControlledModelParamConfig(k, i, ...
    [], false, ... % we integrate the measurement uncertainty in the state-processor
    [], false, ...
    inp_dist, true, ...
    ref_state_builder, true, f, weights_size);

create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, @pd_controller, ...
    k, i, params, dyn_conf), 3 * k.get_size(), dyn_conf.get_params_size());
max_half_width = params.max_road_width / 2;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTTrailerMirroring(k);
dt_controller = 0.01;
terminal_cond = StandstillTerminalCondition(k, 1);
occ_comp = create_kst_trailer_occ_comp(k, params);

debug = false;
create_plot = false;
show_plot = false;
save_plot = false;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, ...
    max_half_width, ...
    constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 2, 0.1, ...
    terminal_cond, ...
    [], ...
    false, false, ...
    [], ...
    occ_comp, ...
    debug, create_plot, show_plot, save_plot, do_random_simulations);

terminal_cond = StandstillTerminalCondition(k, 1);
simulator = Simulator(terminal_cond, ...
    settings.dt_control, settings.time_decimal_places, model_for_ref_traj, k, i, params, constraints, ...
    false, false);
success = simulator.simulate(traj, center(start_set.cora_set), total_duration);

if ~success
    error("Invalid trajectory");
end

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;
sim = StandardSimulator(create_dynamics(), dyn_conf);
gen = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

dyn = create_dynamics();

p = Primitive(start_set, traj, dt_controller, 0.1);

log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
log_fid = fopen(log_file_path, 'wt');

res = gen.generate(settings, p, dyn, log_fid);

return  %% FIXME only for profiling purposes

num_steps = length(res.reachsets);

% Plot truck at iniial state
truck_plotter = KSTTrailerPlotter(occ_comp, k, params);
truck_plotter.plot_vehicle(center(start_set.cora_set), 'r');
hold on;
axis equal;

% Plot trajectory
X = p.ref_traj.states(k.X, 1:num_steps);
Y = p.ref_traj.states(k.Y, 1:num_steps);
plot(X, Y, 'color', 'red', 'LineWidth', 2);
hold on;

% Plot some reachsets
color_map_truck = @gray;
color_map_trailer = @copper;
num_occs_to_merge = 100;
num_occs_sets = ceil(length(res.reachsets) / num_occs_to_merge);
c_map_truck = color_map_truck(num_occs_sets);
c_map_trailer = color_map_trailer(num_occs_sets);
c_maps = {c_map_truck, c_map_trailer};
% for i = 1:num_occs_sets
for i = 2:num_occs_sets
    % start_idx = (i - 1) * num_occs_to_merge + 1;
    % end_idx = min(i * num_occs_to_merge, length(res.reachsets));
    start_idx = (i - 1) * num_occs_to_merge + 1;
    end_idx = start_idx;
    merged_occs = occ_comp.compute_accumulated_occupancy_for_multiple_time_intervals( ...
        res.reachsets, start_idx, end_idx);
    for j = 1:occ_comp.num_shapes
        % we compute the vector to the top or bottom of the shape
        [itv_rot, itv_back_rot] = fit_with_interval(merged_occs{j});

        half_width = min(rad(itv_rot));
        if half_width <= max_half_width + 1.0 % FIXME remove tolerance
            c_map = c_maps{j};
            c = c_map(i, :);

            plot(itv_back_rot, [1 2], '--', 'color', c, 'LineWidth', 1);
            hold on;
        end
    end
end

hold on;

% TODO take final set from p_1 and create a new primitive with the other half

function fun = create_acc_function(acc_profile, duration)
    dt_per_acc = duration / length(acc_profile);

    function a = get_acc(t)
        idx = min(floor(t / dt_per_acc) + 1, length(acc_profile));
        a = acc_profile(idx);
    end

    fun = @get_acc;
end
