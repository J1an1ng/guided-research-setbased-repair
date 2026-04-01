classdef KSTTrailerInpDeltaStateOrder < KSTInpDeltaStateOrder
    % State order for kinematic single track model
    properties (SetAccess = immutable, GetAccess = public)
        HA uint32 % hitch angle
    end

    properties (Constant)
        SIZE = 5
    end

    methods (Static)

        function size = get_size()
            size = 5;
        end

        function order = default_order()
            order = KSTInpDeltaStateOrder.default_order();
            order.HA = 5;
        end

    end

    methods

        function obj = KSTTrailerInpDeltaStateOrder(order)
            if ~exist('order', 'var') || isempty(order)
                order = KSTTrailerInpDeltaStateOrder.default_order();
            end
            obj = obj@KSTInpDeltaStateOrder(order);
            obj.HA = order.HA;
        end

        function label = label_from_state_idx(obj, idx)
            if idx == obj.HA
                label = 'HA';
            else
                label = label_from_state_idx@KSTInpDeltaStateOrder(obj, idx);
            end
        end

        function unit_string = unit_string_from_state_idx(obj, idx)
            if idx == obj.HA
                unit_string = 'rad';
            else
                unit_string = unit_string_from_state_idx@KSTInpDeltaStateOrder(obj, idx);
            end
        end

    end
end
