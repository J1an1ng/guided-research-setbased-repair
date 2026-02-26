classdef (Abstract) AbstractTerminalCondition < handle & matlab.mixin.Heterogeneous
    methods (Abstract)
        is_terminal = check_for_state_hard(obj, state)
        is_terminal = check_for_state_soft(obj, state)
        is_terminal = check_for_set_soft(obj, reachset)
        is_terminal = check_for_itv_soft(obj, itv)
    end
end
