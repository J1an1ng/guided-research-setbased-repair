classdef (Abstract) CostDef < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        cost = evaluate(obj, primitive, reach_res)
    end
end
