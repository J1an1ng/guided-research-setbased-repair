classdef PointMassLongInpTrajGenerator < AbstractInpTrajGenerator
    %% Samples trajectories that reach points equally distributed on a
    %% halfcircle in front of the ego vehicle.
    properties (Access = private)
        k PointMassLongStateOrder
        i InpOrderA

        accelerations cell
    end

    methods

        function obj = PointMassLongInpTrajGenerator( ...
                state_order, inp_order, accelerations)
            obj.k = state_order;
            obj.i = inp_order;
            obj.accelerations = accelerations;
        end

        function trajectories = generate(obj, ~)
            trajectories = cellfun(@(a) PointMassLongInpTraj(obj.k, obj.i, a), ...
                obj.accelerations, 'UniformOutput', false);
        end

    end
end
