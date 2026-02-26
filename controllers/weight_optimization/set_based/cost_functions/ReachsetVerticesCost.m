classdef ReachsetVerticesCost < CostDef
    properties (Access = private)
        get_max_itv
        freq uint32
        settings PrimitiveGenerationSettings
        dim_weights (:, 1) double
    end

    methods

        function obj = ReachsetVerticesCost(get_max_itv, freq, settings, dim_weights)
            obj.get_max_itv = get_max_itv;
            obj.freq = freq;
            obj.settings = settings;
            obj.dim_weights = dim_weights;
        end

        function cost = evaluate(obj, primitive, reach_res)
            reachsets = reach_res.reachsets(1:obj.freq:end);
            ref_states = primitive.ref_traj.states(:, 1:obj.freq:end);
            n = length(reachsets);
            start_set = interval(primitive.start_set.cora_set);
            costs = arrayfun(@(i) i * obj.compute_cost_for_time_step(reachsets(i), ...
                ref_states(:, i), start_set), 1:n);
            costs = costs / n;
            cost = mean(costs);
        end

        function cost = compute_cost_for_time_step(obj, reachset, ref_state, start_set)
            r = interval(reachset.timePoint.set{end});
            c = center(r);
            max_itv = interval(obj.get_max_itv(c));
            rads = rad(max_itv);
            rads_start = rad(start_set);
            rads = max(rads, rads_start);
            verts = vertices(r);
            dists = abs(verts - ref_state);
            max_dist_per_dim = max(dists, [], 2);
            relative_max_dist_per_dim = max_dist_per_dim ./ rads;
            relative_max_dist_per_dim = relative_max_dist_per_dim .* obj.dim_weights;
            cost = mean(arrayfun(@err_fun, relative_max_dist_per_dim));
        end

    end
end

function err = err_fun(x)
    err = x * x;
end
