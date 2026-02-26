function plot_simulations(settings, p, sim_res, dims, occ_freq)
    assert(length(dims) == 2);

    k = settings.k;
    occ_comp = settings.occ_comp;

    plot(p.ref_traj.states(dims(1), :), p.ref_traj.states(dims(2), :), 'Color', 'green', 'LineWidth', 3);
    hold on;

    for i = 1:length(sim_res)
        sim_i = sim_res(i);
        for j = 1:length(sim_i.x)
            plot(sim_i.x{j}(:, dims(1)), sim_i.x{j}(:, dims(2)), 'Color', 'r', 'LineWidth', 2);
            hold on;
        end
    end

    for i = occ_freq:occ_freq:length(p.ref_traj.states)
        % for i = [length(p.ref_traj.states)]
        o = occ_comp.compute_occupancy_for_single_state(p.ref_traj.states(:, i));
        for j = 1:length(o)
            plot(o{j}, [1 2], 'color', 'green', 'LineWidth', 3);
            hold on;
        end
    end

    xlabel(k.label_from_state_idx(dims(1)));
    ylabel(k.label_from_state_idx(dims(2)));

    hold on;

    for i = 1:length(sim_res)
        sim_i = sim_res(i);
        for j = 1:length(sim_i.x)
            if mod(j, occ_freq) == 0
                % if j == length(sim_i.x)
                o = occ_comp.compute_occupancy_for_single_state(sim_i.x{j}(end, 1:6)');
                for k = 1:length(o)
                    plot(o{k}, [1 2], 'color', 'red', 'LineWidth', 2);
                    hold on;
                end
            end
        end
    end

    axis equal;
end
