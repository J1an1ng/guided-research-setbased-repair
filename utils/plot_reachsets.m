function f = plot_reachsets(ref_traj, reachsets, k, cols, sim_res)
    f = figure;
    tiledlayout(cols, ceil(k.get_size() / cols));

    for i = 1:k.get_size()
        nexttile;
        if ~isempty(reachsets)
            plotOverTime(reachsets, i, 'LineWidth', 3.0);
        end

        if ~isempty(sim_res)
            hold on;

            for j = 1:length(sim_res)
                sim_res_t = [sim_res(j).t{:}];
                sim_res_x = cell2mat(sim_res(j).x);
                plot(sim_res_t, sim_res_x(:, i), 'Color', 'r', 'LineWidth', 2);
            end
        end

        hold on;
        plot(ref_traj.times, ref_traj.states(i, :), 'Color', 'yellow');

        dt = ref_traj.duration / ref_traj.num_steps;
        ext_horiz_len = size(ref_traj.ext_horizon_states, 2);
        if ext_horiz_len > 0
            start_time = ref_traj.times(end);
            end_time = start_time + ext_horiz_len * dt;
            plot(start_time + dt:dt:end_time, ref_traj.ext_horizon_states(i, :), 'Color', 'yellow');
        end

        title(k.label_from_state_idx(i), 'FontSize', 22);
        xlabel('s', 'FontSize', 18);
        ylabel(k.unit_string_from_state_idx(i), 'FontSize', 18);
        set(get(gca, 'ylabel'), 'rotation', 0);
    end
end
