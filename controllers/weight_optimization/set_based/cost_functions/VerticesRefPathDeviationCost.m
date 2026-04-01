classdef VerticesRefPathDeviationCost < CostDef
    properties (Access = private)
        k KSTStateOrder
        weight_lon double
        weight_lat double
    end

    methods

        function obj = VerticesRefPathDeviationCost(state_order, weight_lon, weight_lat)
            obj.k = state_order;
            obj.weight_lon = weight_lon;
            obj.weight_lat = weight_lat;
        end

        function cost = evaluate(obj, primitive, reach_res)
            ref_states = primitive.ref_traj.states;
            actual_reachsets = reach_res.reachsets;
            num_steps = length(actual_reachsets);
            % TODO is the weighting sensible??  --> try
            dists = arrayfun(@(i) (i / num_steps) * obj.compute_distance_to_ref_for_sets(ref_states(:, i), ...
                actual_reachsets(i)), 1:num_steps);
            cost = mean(dists);
        end

    end

    methods (Access = private)

        function dist = compute_distance_to_ref_for_sets(obj, ref_state, reachset)
            dists = cellfun(@(s) obj.compute_distance_to_ref_for_single_set(ref_state, s), ...
                reachset.timePoint.set);
            % dist = mean(exp(10 * dists + 1));
            dist = mean(dists);
        end

        function dist = compute_distance_to_ref_for_single_set(obj, ref_state, reachset)
            actual_set_proj = project(reachset, [obj.k.X obj.k.Y obj.k.PSI]);
            actual_set_proj = reduce(actual_set_proj, 'pca', 3);
            actual_set_verts = vertices(actual_set_proj);
            dist = mean(arrayfun(@(i) obj.compute_distance_to_ref(ref_state, actual_set_verts(:, i)), ...
                1:size(actual_set_verts, 2)));
        end

        function dist = compute_distance_to_ref(obj, ref_state, state)
            [dist_lon, dist_lat] = get_pos_deviation(ref_state(obj.k.PSI), ...
                    ref_state(obj.k.X), ref_state(obj.k.Y), ...
                    state(obj.k.X), state(obj.k.Y));
            dist_lon = abs(dist_lon);
            dist_lat = abs(dist_lat);
            dist = obj.weight_lon * dist_lon + obj.weight_lat * dist_lat;
        end

    end
end
