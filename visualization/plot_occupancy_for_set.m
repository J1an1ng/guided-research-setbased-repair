function plot_occupancy_for_set(occ_comp, reach_set, plot_occ_of_vertices, state_order_for_occ)
    f_occ = figure('Visible', 'on');
    hold on;

    % Plot occupancy computed from set
    occs = occ_comp.compute_occupancy(reach_set);
    for i = 1:length(occs)
        plot(occs{i}, [1 2], '--', 'Color', 'r');
        axis equal;
    end

    if plot_occ_of_vertices
        state_order = occ_comp.k;
        dim_mapping_down = state_order_for_occ.create_state_order_mapping(state_order);
        reach_set_proj = project(reach_set, dim_mapping_down);

        % We need the reduction, otw vertices exceeds memory and takes ages
        % reach_set_proj_red = reduce(reach_set_proj, 'pca', 200);
        % verts = vertices(reach_set_proj_red);
        verts = [];
        for k = 1:1
            verts = [verts randPoint(reach_set_proj, 10000, 'extreme')];
        end
        fprintf('#vertices: %d\n', size(verts, 2));
        figure('Visible', 'on');
        hold on;
        plot_combs = nchoosek(1:state_order_for_occ.SIZE, 2);
        cols = 3;
        tiledlayout(cols, ceil(size(plot_combs, 1) / cols));

        for i = 1:size(plot_combs, 1)
            nexttile;
            hold on;
            combs = plot_combs(i, :);
            plot(reach_set_proj, combs, 'Color', 'b');
            plot(verts(combs(1), :), verts(combs(2), :), '.');
            title(sprintf('%d/%d', combs(1), combs(2)));
            xlabel(sprintf('dim %d', combs(1)));
            ylabel(sprintf('dim %d', combs(2)));
        end

        figure(f_occ);
        % for i = 1:size(verts, 2)
        % v = verts(:, i);
        % v_ext = zeros(state_order.SIZE, 1);
        % v_ext(dim_mapping_down) = v;

        % plot(v_ext(1), v_ext(1), "o", 'Color', 'b');
        % occ_for_v = occ_comp.computae_occupancy_for_single_state(v_ext);
        % for k = 1:length(occ_for_v)
        % plot(occ_for_v{k}, [1 2], 'Color', 'g');
        % axis equal;
        % end

        % end
        v_ext = zeros(state_order.SIZE, size(verts, 2));
        v_ext(dim_mapping_down, :) = verts;
        occs_for_v = arrayfun(@(i) occ_comp.compute_occupancy_for_single_state( ...
            v_ext(:, i)), 1:size(v_ext, 2), 'UniformOutput', false);

        num_shapes = size(occs_for_v{1}, 1);
        for i = 1:num_shapes
            shapes = cellfun(@(s) s{i}, occs_for_v, 'UniformOutput', false);
            shapes_union = fold(@(a, b) a | b, shapes);
            plot(shapes_union, [1 2], 'Color', 'g');
        end
    end

    axis equal;
end
