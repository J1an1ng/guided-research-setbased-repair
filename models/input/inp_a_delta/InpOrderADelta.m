classdef InpOrderADelta < AbstractInpOrder
    properties (SetAccess = immutable, GetAccess = public)
        A uint32
        DELTA uint32
    end

    properties (Constant)
        SIZE = 2
    end

    methods (Static)

        function order = default_order()
            order.DELTA = 1;
            order.A = 2;
        end

    end

    methods

        function obj = InpOrderADelta(order)
            if ~exist('order', 'var') || isempty(order)
                order = InpOrderADelta.default_order();
            end
            obj.DELTA = order.DELTA;
            obj.A = order.A;
        end

    end
end
