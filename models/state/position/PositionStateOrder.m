classdef PositionStateOrder < AbstractStateOrder
    properties (Constant)
        X = 1
        Y = 2
    end

    methods (Static)

        function size = get_size()
            size = 2;
        end

    end

    methods

        function label = label_from_state_idx(obj, idx)
            switch idx
                case obj.X
                    label = 'X';
                case obj.Y
                    label = 'Y';
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
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

    end
end
