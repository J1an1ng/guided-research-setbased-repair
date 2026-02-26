classdef (Abstract) AbstractDistProcessor < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        measured_state = process_meas_dist(obj, state, meas_dist)
        dx = process_model_add_dist(obj, state, dx, model_add_dist)
    end
end
