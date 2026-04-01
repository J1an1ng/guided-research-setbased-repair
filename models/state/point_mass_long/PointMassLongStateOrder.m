classdef PointMassLongStateOrder < AbstractStateOrder
    properties (SetAccess = immutable, GetAccess = public)
        S uint32
        V uint32
    end

    properties (Constant)
        SIZE = 2
    end

    methods (Static)

        function size = get_size()
            size = 2;
        end

        function order = default_order()
            order.S = 1;
            order.V = 2;
        end

    end

    methods

        function obj = PointMassLongStateOrder(order)
            if ~exist('order', 'var') || isempty(order)
                order = PointMassLongStateOrder.default_order();
            end
            obj.S = order.S;
            obj.V = order.V;
        end

        function label = label_from_state_idx(obj, idx)
            switch idx
                case obj.S
                    label = 'S';
                case obj.V
                    label = 'V';
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

        function unit_string = unit_string_from_state_idx(obj, idx)
            switch idx
                case obj.S
                    unit_string = 'm';
                case obj.V
                    unit_string = 'm/s';
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

    end
end
