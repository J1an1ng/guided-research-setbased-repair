classdef RefPathDeviationCost < CostDef
    properties (Access = private)
        k KSTStateOrder
    end

    methods

        function obj = RefPathDeviationCost(state_order)
            obj.k = state_order;
        end

        function cost = evaluate(obj, primitive, reach_res)
            ref_states = primitive.ref_traj.states;
            actual_reachsets = reach_res.reachsets;
            pos_dims = [obj.k.X, obj.k.Y];
            dists = arrayfun(@(i) compute_distance_to_ref_for_sets(ref_states(:, i), ...
                actual_reachsets(i), pos_dims), 1:length(actual_reachsets));
            cost = mean(dists);
        end

    end
end

function dist = compute_distance_to_ref_for_sets(ref_state, reachset, position_dims)
    ref_pos = ref_state(position_dims);
    dists = cellfun(@(s) compute_distance_to_ref_for_single_set(ref_pos, s, position_dims), ...
        reachset.timePoint.set);
    dist = mean(dists);
end

function dist = compute_distance_to_ref_for_single_set(ref_state, reachset, position_dims)
    ref_pos = ref_state(position_dims);
    actual_pos_set = project(reachset, position_dims);
    actual_pos = center(actual_pos_set);
    dist = norm(ref_pos - actual_pos);
    dist = err_fun(dist);
end

function err = err_fun(x)
    err = x;
end
