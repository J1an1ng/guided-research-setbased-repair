classdef PointMassLongInpTraj < AbstractInpTraj
    properties (Constant)
        id_properties = ["a"]
    end

    properties (SetAccess = immutable, GetAccess = public)
        k PointMassLongStateOrder
        i InpOrderA
        a double
    end

    methods

        function obj = PointMassLongInpTraj(k, i, a)
            obj.k = k;
            obj.i = i;
            obj.a = a;
        end

        function inp = inp_at(obj, state, ~, dt)
            v = state(obj.k.V);
            inp_a_stop = -v / dt;
            inp_a = max(obj.a, inp_a_stop);

            %% TODO call repair_inp on constraints as soon as they are implemented

            inp = zeros(obj.i.SIZE, 1);
            inp(obj.i.A) = inp_a;
        end

    end
end
