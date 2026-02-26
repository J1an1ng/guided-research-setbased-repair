classdef SimRefPathDeviationCost < SimCostDef
    properties (Access = private)
        dims_for_cost (:, 1) uint32
        scale_per_dim (:, 1) double
    end

    methods

        function obj = SimRefPathDeviationCost(dims_for_cost, weights_per_dim)
            assert(length(dims_for_cost) == length(weights_per_dim), ...
                'Dimensions for cost and weights must have the same length');
            obj.dims_for_cost = dims_for_cost;
            obj.scale_per_dim = weights_per_dim ./ sum(weights_per_dim);
        end

        function cost = evaluate(obj, ref_traj, single_sim_res)
            err = single_sim_res.error{1};
            if ~isempty(err)
                assert(err.identifier == "CONTROL:input_violates_bounds");
                cost = sum(exp(5 * err.bound_violations));
                return
            end

            x = cell2mat(single_sim_res.x);

            costs = arrayfun(@(i) obj.compute_cost_for_time_step(ref_traj.states(:, i), x(i, :)'), 1:size(x, 1));
            cost = mean(costs);
        end

    end

    methods (Access = private)

        function cost = compute_cost_for_time_step(obj, ref_state, sim_state)
            cost = norm(obj.scale_per_dim .* (ref_state(obj.dims_for_cost) - sim_state(obj.dims_for_cost)));
        end

    end
end
