classdef InpCreatorForADelta < AbstractInpCreator
    properties (Access = private)
        state_order KSTInpDeltaStateOrder
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
            inp = create_inp_for_a_delta([], obj.inp_order, [], a, desired_delta, []);
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
