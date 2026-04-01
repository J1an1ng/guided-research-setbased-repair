function create_occupancy_plot(occ_comp, k, params, initial_state, ref_traj, res, max_half_width, num_occs_to_merge)
    figure;
    num_steps = length(res.reachsets);
    % Plot truck at iniial state
    truck_plotter = KSTPlotter(occ_comp, k, params);
    truck_plotter.plot_vehicle(initial_state, 'r');
    hold on;
    axis equal;

    % Plot trajectory
    X = ref_traj.states(k.X, 1:num_steps);
    Y = ref_traj.states(k.Y, 1:num_steps);
    plot(X, Y, 'color', 'red', 'LineWidth', 2);
    hold on;

    % Plot some occupancies
    color_map_truck = @gray;
    color_map_trailer = @copper;
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

    xlabel('m', 'FontSize', 18);
    ylabel('m', 'FontSize', 18);
    set(get(gca, 'ylabel'), 'rotation', 0);
    ylim([-7, 4]);
end
