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

dt_controller = 0.01;

terminal_cond = StandstillTerminalCondition(k, 1);
time_decimal_places = 2;

constraints = KSTTrailerConstraints(k, i, veh_params);

%% Initial state
v = 15.0;
initial_state = zeros(k.get_size(), 1);
initial_state(k.V) = v;

%% Reference trajectory
a = -1.0;
pp_time_constant = 0.05;

inp_creator = InpCreatorForADeltaV(k, i, veh_params, pp_time_constant);

total_duration = abs(v / a);
total_dist = abs((v^2) / (2 * a));

lane_width = 50.0;
shoulder_width = 50.0;
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
    0.0, @(x, ~) x);
success = ref_traj_sim.simulate(traj, initial_state, total_duration, dt_controller);

if ~success
    warning("Invalid trajectory");
end

occ_comp = create_kst_trailer_occ_comp(k, veh_params);
truck_plotter = KSTPlotter(occ_comp, k, veh_params);

fig = uifigure("Position", [100 100 300 250]);
g = uigridlayout(fig);
g.RowHeight = {'1x', 'fit'};
g.ColumnWidth = {'1x', '1x', '1x'};

% Axes for plotting the truck
ax = uiaxes(g);
ax.Layout.Row = 1;
ax.Layout.Column = [1 3];

sld = uislider(g, "ValueChangedFcn", @(src, event) plot_vehicle_at_state(src, event, fig, ax, truck_plotter, traj));
sld.Limits = [0 size(traj.states, 2)];
sld.Layout.Row = 2;
sld.Layout.Column = 2;

init_event.Value = 1;
plot_vehicle_at_state([], init_event, fig, ax, truck_plotter, traj);

function plot_vehicle_at_state(src, event, fig, ax, truck_plotter, traj)
    y = event.Value;

    cla(ax);

    plot(ax, traj.states(1, :), traj.states(2, :));

    hold(ax, 'on');
    % plot(interval([0; 0], [20, 5]), [1 2], 'Parent', ax, 'FaceColor', 'yellow');
    truck_plotter.plot_vehicle(traj.states(:, max(1, int32(y))), 'r', ax);
    axis(ax, 'equal');
    ax.YLim = [traj.states(2, end) - 3 traj.states(2, 1) + 3];
    ax.XLim = [traj.states(1, 1) - 10 traj.states(1, end) + 10];
    close; % hack as Cora creates figure...

    % axis equal;
    %
    % Optionally auto-scale Y axis if you want, else comment this out:
    % ax.YLim = [min(ax.YLim(1), y-0.5), max(ax.YLim(2), y+0.5)];
end
