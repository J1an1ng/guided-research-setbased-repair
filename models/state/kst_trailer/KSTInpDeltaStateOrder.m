classdef KSTInpDeltaStateOrder < AbstractKSTStateOrder
    % State order for kinematic single track model
    properties (SetAccess = immutable, GetAccess = public)
        X uint32
        Y uint32
        V uint32 % velocity
        PSI uint32 % orientation
    end

    methods (Static)

        function size = get_size()
            size = 4;
        end

        function order = default_order()
            order.X = 1;
            order.Y = 2;
            order.V = 3;
            order.PSI = 4;
        end

    end

    properties (Constant)
        SIZE = 4
    end

    methods

        function obj = KSTInpDeltaStateOrder(order)
            if ~exist('order', 'var') || isempty(order)
                order = KSTInpDeltaStateOrder.default_order();
            end
            obj.X = order.X;
            obj.Y = order.Y;
            obj.V = order.V;
            obj.PSI = order.PSI;
        end

        function label = label_from_state_idx(obj, idx)
            switch idx
                case obj.X
                    label = 'X';
                case obj.Y
                    label = 'Y';
                case obj.V
                    label = 'V';
                case obj.PSI
                    label = 'PSI';
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

        function unit_string = unit_string_from_state_idx(obj, idx)
            switch idx
                case obj.X
                    unit_string = 'm';
                case obj.Y
                    unit_string = 'm';
                case obj.V
                    unit_string = 'm/s';
                case obj.PSI
                    unit_string = 'rad';
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

    end
end
