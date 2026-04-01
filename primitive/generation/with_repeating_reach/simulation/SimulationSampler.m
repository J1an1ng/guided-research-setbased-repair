classdef (Abstract) SimulationSampler < handle & matlab.mixin.Heterogeneous
    methods (Abstract, Access = public)
        x = sample_initial_state(obj, initial_set)
        meas_dist = sample_meas_dist(obj, state, ref_state)
        model_add_dist = sample_model_add_dist(obj, state, ref_state)
        inp_dist = sample_inp_dist(obj, state, ref_state)
    end
end
