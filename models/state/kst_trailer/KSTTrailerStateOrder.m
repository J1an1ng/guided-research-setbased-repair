classdef KSTTrailerStateOrder < KSTStateOrder
    properties (SetAccess = immutable, GetAccess = public)
        HA uint32 % hitch angle
    end

    properties (Constant)
        SIZE = 6
    end

    methods (Static)

        function size = get_size()
            size = 6;
        end

        function order = default_order()
            order = KSTStateOrder.default_order();
            order.HA = 6;
        end

    end

    methods

        function obj = KSTTrailerStateOrder(order)
            if ~exist('order', 'var') || isempty(order)
                order = KSTTrailerStateOrder.default_order();
            end
            obj = obj@KSTStateOrder(order);
            obj.HA = order.HA;
        end

        function label = label_from_state_idx(obj, idx)
            if idx == obj.HA
                label = '\alpha';
            else
                label = label_from_state_idx@KSTStateOrder(obj, idx);
            end
        end

        function unit_string = unit_string_from_state_idx(obj, idx)
            if idx == obj.HA
                unit_string = 'rad';
            else
                unit_string = unit_string_from_state_idx@KSTStateOrder(obj, idx);
            end
        end

    end
end
