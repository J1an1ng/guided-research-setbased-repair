classdef (Abstract) AbstractInpCreator < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        inp = create_inp(obj, state, a, desired_delta)

        inp = create_max_inp(obj, a, direction) % direction: 1 (left), 0 (straight), -1 (right)
    end
end
