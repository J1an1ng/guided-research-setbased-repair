classdef (Abstract) AbstractInpTrajGenerator < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        trajectories = generate(obj, initial_state)
        %% the trajectories can be specific to an initial state interval
    end
end
