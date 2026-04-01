classdef KSTConstantInpTrajADeltaV < AbstractInpTraj
    properties (Constant)
        id_properties = ["a"; "delta_v"]
    end

    properties (SetAccess = private, GetAccess = public)
        state_order KSTStateOrder
        inp_order InpOrderADeltaV

        a double {mustBeNonpositive}
        delta_v double
    end

    methods

        function obj = KSTConstantInpTrajADeltaV(state_order, inp_order, a, delta_v)
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.a = a;
            obj.delta_v = delta_v;
        end

        function inp = inp_at(obj, state, ~, dt)
            v = state(obj.state_order.V);

            if v <= 0
                %% the trajectory stops if vehicle stops
                inp = [];
                return
            end

            inp_a = obj.a;
            inp_a_stop = -v / dt;
            inp_a = max(inp_a, inp_a_stop);

            inp = zeros(obj.inp_order.SIZE, 1);
            inp(obj.inp_order.DELTA_V) = obj.delta_v;
            inp(obj.inp_order.A) = inp_a;
        end

        function str = to_string(obj)
            str = sprintf('dv: %0.5g, a: %s', obj.delta_v, obj.a);
        end

    end
end
