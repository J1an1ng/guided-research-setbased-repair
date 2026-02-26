classdef MaximumSetComputer < handle
    properties (Access = private)
        state_order AbstractStateOrder
        % dimensions that have influence on the occupancy
        relevant_dims (1, :) uint32
        max_dir_length_comp MaximumDirectionLengthComputer
        out_file string
        occ_comp OccupancyComputer %% FIXME remove again
    end

    properties (Constant)
        % directions_per_dim = [-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1]
        directions_per_dim = [-1, -0.875, -0.75, -0.625, -0.5, -0.375, -0.25, ...
                              -0.125, 0, 0.125, 0.25, 0.325, 0.5, 0.625, 0.75, 0.825, 1]
    end

    methods

        function obj = MaximumSetComputer(k, relevant_dims, occ_comp, ...
                max_values_per_dim, max_size_per_occ, out_file)
            obj.state_order = k;
            obj.relevant_dims = relevant_dims;
            obj.max_dir_length_comp = MaximumDirectionLengthComputer(occ_comp, k, ...
                relevant_dims, max_values_per_dim, max_size_per_occ);
            obj.out_file = out_file;
            obj.occ_comp = occ_comp;
        end

        function compute_max_set(obj, c)
            direction_vectors = obj.create_direction_vectors();

            direction_vectors = direction_vectors(:, all(direction_vectors([1 4], :) == 0, 1));

            % direction_vectors = datasample(direction_vectors, 300, 2);

            num_directions = size(direction_vectors, 2);

            if isfile(obj.out_file)
                load(obj.out_file, 'max_scales');
            else
                max_scales = zeros(num_directions, 1);

                comp = parallel.pool.Constant(obj.max_dir_length_comp);
                parfor k = 1:num_directions
                    direction = direction_vectors(:, k);
                    fun = comp.Value.create_obj_fun(c, direction);
                    res = fmincon(fun, 1, [-1; 1], [0; 1]);
                    max_scales(k) = res;
                end

                save(obj.out_file, 'max_scales');
            end

            %% FIXME extend halfspace_lengths as well and check if other dims are handled correctly
            % p = c + polytope([direction_vectors -direction_vectors]', ...
            % [halfspace_lengths; halfspace_lengths]);
            % pp = projectHighDim(p, obj.state_order.SIZE, obj.relevant_dims);
            % cz = conZonotope(pp);
            scaled_dir_vectors = direction_vectors .* max_scales';

            plot_vertices(scaled_dir_vectors, direction_vectors, [2 3]);

            p = polytope([scaled_dir_vectors, -scaled_dir_vectors]);
            cz = conZonotope(p);
            z = zonotope(p);

            plot_vertices(scaled_dir_vectors, direction_vectors, [2 3], p, cz, z);
            plot_vertices(scaled_dir_vectors, direction_vectors, [3 4], p, cz, z);

            Q = fit_inner_ellipsoid(scaled_dir_vectors, c);
            el = ellipsoid(Q, c);

            %% TODO check if this is correct
            zz = zonotope(el, 20, 'inner:norm');

            display(zz);

            p = polytope([scaled_dir_vectors, -scaled_dir_vectors]);

            plot(p, [2 3]);
            hold on;
            for i = 1:size(scaled_dir_vectors, 2)
                if scaled_dir_vectors(1, i) == 0 && scaled_dir_vectors(4, i) == 0
                    plot(scaled_dir_vectors(2, i), scaled_dir_vectors(3, i), '.', 'MarkerSize', 12);
                    plot(-scaled_dir_vectors(2, i), -scaled_dir_vectors(3, i), '.', 'MarkerSize', 12);
                end
            end

            pp = projectHighDim(p, obj.state_order.get_size(), obj.relevant_dims);
            cz = conZonotope(pp);

            occ = obj.occ_comp.compute_occupancy(cz);
            truck_occ = occ{1};
            trailer_occ = occ{2};
            plot(truck_occ, [1 2]);
            hold on;
            plot(trailer_occ, [1 2]);
        end

    end

    methods (Access = private)

        function direction_vectors = create_direction_vectors(obj)
            direction_vectors = get_all_combinations(obj.directions_per_dim, length(obj.relevant_dims));
            % We remove the 0 entry...
            direction_vectors(:, all(direction_vectors(:, :) == 0)) = [];
            % ...and norm all vectors...
            direction_vectors = direction_vectors ./ vecnorm(direction_vectors);
            % ...also w.r.t. their direction of the first dimension.
            direction_vectors_sign_of_first = (2 * (direction_vectors(1, :) >= 0) - 1);
            direction_vectors = direction_vectors .* direction_vectors_sign_of_first;

            % We remove duplicates (note that we have to transpose the matrix first
            % to make uniquetol work correctly and then transpose the result back)
            direction_vectors = uniquetol(direction_vectors', 1e-6, 'byrows', true)';
        end

    end

end

function plot_vertices(scaled_dir_vectors, direction_vectors, dims, p, cz, z)
    figure;
    hold on;
    other_dims = setdiff(1:size(scaled_dir_vectors, 1), dims);
    for k = 1:size(scaled_dir_vectors, 2)
        v_scaled = scaled_dir_vectors(:, k);
        v = direction_vectors(:, k);

        if all(v(other_dims) == 0)

            v_1 = v_scaled(dims(1));
            v_2 = v_scaled(dims(2));
            plot(v_1, v_2, '.', 'MarkerSize', 20, 'Color', 'black');
            plot(-v_1, -v_2, '.', 'MarkerSize', 20, 'Color', 'black');

            v_1 = v(dims(1));
            v_2 = v(dims(2));
            plot(v_1, v_2, '.', 'MarkerSize', 20, 'Color', 'green');
            plot(-v_1, -v_2, '.', 'MarkerSize', 20, 'Color', 'green');
        end
    end
    plot(p, dims, 'Color', 'black');
    plot(cz, dims, 'Color', 'red');
    plot(z, dims, 'Color', 'green');
end
