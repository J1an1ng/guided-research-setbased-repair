classdef KSTTrailerStateOrderOnlyAbs < AbstractStateOrder
    properties (Constant)
        DELTA = 1
        V = 2
        HA = 3
        SIZE = 3
    end

    methods

        function size = get_size(obj)
            size = obj.SIZE;
        end

        function label = label_from_state_idx(obj, idx)
            switch idx
                case obj.DELTA
                    label = 'DELTA';
                case obj.V
                    label = 'V';
                case obj.HA
                    label = 'HA';
                otherwise
                    error('Unknown state index %d', idx);
            end
        end

    end
end
