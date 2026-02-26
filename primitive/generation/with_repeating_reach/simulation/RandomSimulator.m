classdef (Abstract) RandomSimulator < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        simulate(obj, initial_set, ref_traj, t_start, t_end)
    end
end
