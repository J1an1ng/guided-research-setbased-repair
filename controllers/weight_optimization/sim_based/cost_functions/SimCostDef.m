classdef (Abstract) SimCostDef < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        cost = evaluate(obj, ref_traj, single_sim_res)
    end
end
