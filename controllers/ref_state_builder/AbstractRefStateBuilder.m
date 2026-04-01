classdef (Abstract) AbstractRefStateBuilder < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        ref_state_size = get_ref_state_size(obj)
        ref_state = build_ref_state(obj, ref_states, ref_state_args, ext_horizon_ref_states, time_step)
    end
end
