classdef CombinedCost < CostDef
    properties (Access = private)
        cost_defs CostDef
        weights (:, 1) double
    end

    methods

        function obj = CombinedCost(cost_defs, weights)
            obj.cost_defs = cost_defs;
            obj.weights = weights;
            assert(length(obj.cost_defs) == size(obj.weights, 1), ...
                'The number of cost definitions must match the number of weights');
        end

        function cost = evaluate(obj, primitive, reach_res)
            cost = 0;
            for i = 1:length(obj.cost_defs)
                cost = cost + obj.weights(i) * obj.cost_defs(i).evaluate(primitive, reach_res);
            end
        end

    end
end
