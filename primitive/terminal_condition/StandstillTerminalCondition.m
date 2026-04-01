classdef StandstillTerminalCondition < AbstractTerminalCondition
    properties (Constant)
        v_threshold_hard = 0.2
    end

    properties (Access = private)
        k AbstractStateOrder
        v_threshold double
    end
    methods

        function obj = StandstillTerminalCondition(k, v_threshold)
            obj.k = k;
            assert(isprop(k, 'V'), 'State order must have property V for velocity');
            obj.v_threshold = v_threshold;
        end

        function is_terminal = check_for_state_hard(obj, state)
            is_terminal = state(obj.k.V) <= obj.v_threshold_hard;
        end

        function is_terminal = check_for_state_soft(obj, state)
            is_terminal = state(obj.k.V) <= obj.v_threshold;
        end

        function is_terminal = check_for_set_soft(obj, reachset)
            v_itv = interval(project(reachset, [obj.k.V]));
            is_terminal = v_itv.sup <= obj.v_threshold;
        end

        function is_terminal = check_for_itv_soft(obj, itv)
            v_itv = itv(obj.k.V, :);
            is_terminal = v_itv(2) <= obj.v_threshold;
        end

    end
end
