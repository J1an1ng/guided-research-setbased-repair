classdef InpCreatorForADeltaV < AbstractInpCreator
    properties (Access = private)
        state_order KSTStateOrder
        inp_order InpOrderADeltaV
        params KSTParams
        time_constant % double
    end

    methods

        function obj = InpCreatorForADeltaV(state_order, inp_order, params, time_constant)
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.params = params;
            obj.time_constant = time_constant;
        end

        function inp = create_inp(obj, state, a, desired_delta)
            inp = create_inp_for_a_delta_v(obj.state_order, obj.inp_order, state, a, desired_delta, obj.time_constant);
        end

        function inp = create_max_inp(obj, a, direction)
            inp = zeros(obj.inp_order.SIZE, 1);

            if direction == 1
                delta_v = obj.params.steering_v_max;
            elseif direction == -1
                delta_v = obj.params.steering_v_min;
            elseif direction == 0
                delta_v = 0;
            else
                error('Invalid direction: %d', direction);
            end
            inp(obj.inp_order.DELTA_V) = delta_v;

            inp(obj.inp_order.A) = a;
        end

    end
end
