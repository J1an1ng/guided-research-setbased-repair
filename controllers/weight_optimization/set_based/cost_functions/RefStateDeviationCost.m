classdef RefStateDeviationCost < CostDef
    properties (Access = private)
        settings PrimitiveGenerationSettings
        dim_weights (:, 1) double
    end

    methods

        function obj = RefStateDeviationCost(settings, dim_weights)
            obj.settings = settings;
            obj.dim_weights = dim_weights;
        end

        function cost = evaluate(obj, primitive, reach_res)
            radii = rad(interval(primitive.start_set.cora_set));
            weights = obj.dim_weights ./ radii;

            ref_states = primitive.ref_traj.states;
            actual_reachsets = reach_res.reachsets;
            dists = arrayfun(@(i) compute_distance_to_ref_for_sets(ref_states(:, i), ...
                actual_reachsets(i), weights), 1:length(actual_reachsets));
            cost = mean(dists);
        end

    end
end

function dist = compute_distance_to_ref_for_sets(ref_state, reachset, weights)
    dists = cellfun(@(s) compute_distance_to_ref_for_single_set(ref_state, s, weights), ...
        reachset.timePoint.set);
    dist = mean(dists);
end

function cost = compute_distance_to_ref_for_single_set(ref_state, reachset, weights)
    c = center(reachset);
    dists = c - ref_state;
    dists = dists .* weights;
    cost = mean(arrayfun(@err_fun, dists));
end

function err = err_fun(x)
    err = x;
end
