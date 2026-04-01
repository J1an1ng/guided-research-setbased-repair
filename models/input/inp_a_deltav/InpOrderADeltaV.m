classdef InpOrderADeltaV < AbstractInpOrder
    properties (SetAccess = immutable, GetAccess = public)
        A uint32
        DELTA_V uint32
    end

    properties (Constant)
        SIZE = 2
    end

    methods (Static)

        function order = default_order()
            order.DELTA_V = 1;
            order.A = 2;
        end

    end

    methods

        function obj = InpOrderADeltaV(order)
            if ~exist('order', 'var') || isempty(order)
                order = InpOrderADeltaV.default_order();
            end
            obj.DELTA_V = order.DELTA_V;
            obj.A = order.A;
        end

    end
end
