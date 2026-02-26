classdef KSTDistProcForRotatedPositionalDists < AbstractDistProcessor
    properties (Access = private)
        state_order AbstractKSTStateOrder
    end

    methods (Access = public)

        function obj = KSTDistProcForRotatedPositionalDists(state_order)
            obj.state_order = state_order;
        end

        function measured_state = process_meas_dist(obj, state, meas_dist)
            measured_state = state + transform_dists_to_global_frame(state, obj.state_order, meas_dist);
        end

        function dx = process_model_add_dist(obj, state, dx, model_add_dist)
            dx = dx + transform_dists_to_global_frame(state, obj.state_order, model_add_dist);
        end

    end
end
