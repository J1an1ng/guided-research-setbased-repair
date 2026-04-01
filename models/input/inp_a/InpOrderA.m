classdef InpOrderA < AbstractInpOrder
    properties (SetAccess = immutable, GetAccess = public)
        A uint32
    end

    properties (Constant)
        SIZE = 1
    end

    methods (Static)

        function order = default_order()
            order.A = 1;
        end

    end

    methods

        function obj = InpOrderA(order)
            if ~exist('order', 'var') || isempty(order)
                order = InpOrderA.default_order();
            end
            obj.A = order.A;
        end

    end
end
