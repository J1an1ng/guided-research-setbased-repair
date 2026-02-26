classdef KSTStateOrder < AbstractKSTStateOrder
    % State order for kinematic single track model
    properties (SetAccess = immutable, GetAccess = public)
        X uint32
        Y uint32
        DELTA uint32 % steering angle
        V uint32 % velocity
        PSI uint32 % orientation
    end

    properties (Constant)
        SIZE = 5
    end

    methods (Static)

        function size = get_size()
            size = 5;
        end

        function order = default_order()
            order.X = 1;
            order.Y = 2;
            order.DELTA = 3;
            order.V = 4;
            order.PSI = 5;
        end

    end

    methods

        function obj = KSTStateOrder(order)
            if ~exist('order', 'var') || isempty(order)
                order = KSTStateOrder.default_order();
            end
            obj.X = order.X;
            obj.Y = order.Y;
            obj.DELTA = order.DELTA;
            obj.V = order.V;
            obj.PSI = order.PSI;
        end

        function label = label_from_state_idx(obj, idx)
            switch idx
                case obj.X
                    label = 'x';
                case obj.Y
                    label = 'y';
                case obj.DELTA
                    label = '\delta';
                case obj.V
                    label = 'v';
                case obj.PSI
                    label = '\psi';
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
                case obj.DELTA
                    unit_string = 'rad';
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
