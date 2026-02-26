classdef InpCreatorForADelta < AbstractInpCreator
    properties (Access = private)
        state_order AbstractKSTStateOrder
        inp_order InpOrderADelta
        params KSTParams
    end

    methods

        function obj = InpCreatorForADelta(state_order, inp_order, params)
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.params = params;
        end

        function inp = create_inp(obj, ~, a, desired_delta)
            % inp = zeros(obj.inp_order.SIZE, 1);
            inp(obj.inp_order.DELTA, 1) = desired_delta;
            inp(obj.inp_order.A, 1) = a;
        end

        function inp = create_max_inp(obj, a, direction)
            inp = zeros(obj.inp_order.SIZE, 1);
            if direction == 1
                delta = obj.params.steering_max;
            elseif direction == -1
                delta = obj.params.steering_min;
            elseif direction == 0
                delta = 0;
            else
                error('Invalid direction: %d', direction);
            end
            inp(obj.inp_order.DELTA) = delta;
            inp(obj.inp_order.A) = a;
        end

    end
end
