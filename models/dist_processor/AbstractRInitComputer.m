classdef (Abstract) AbstractRInitComputer < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        r_init = compute_initial_set(obj, measured_state, meas_dist_set)
    end
end
