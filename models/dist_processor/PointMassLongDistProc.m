classdef PointMassLongDistProc < AbstractDistProcessor
    properties (Access = private)
        state_order PointMassLongStateOrder
    end

    methods (Access = public)

        function obj = PointMassLongDistProc(state_order)
            obj.state_order = state_order;
        end

        function measured_state = process_meas_dist(~, state, meas_dist)
            measured_state = state + meas_dist;
        end

        function dx = process_model_add_dist(~, ~, dx, model_add_dist)
            dx = dx + model_add_dist;
        end

    end
end
