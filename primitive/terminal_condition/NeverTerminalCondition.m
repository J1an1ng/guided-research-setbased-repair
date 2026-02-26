classdef NeverTerminalCondition < AbstractTerminalCondition
    methods

        function obj = NeverTerminalCondition()
        end

        function is_terminal = check_for_state_hard(~, ~)
            is_terminal = false;
        end

        function is_terminal = check_for_state_soft(~, ~)
            is_terminal = false;
        end

        function is_terminal = check_for_set_soft(~, ~)
            is_terminal = false;
        end

        function is_terminal = check_for_itv_soft(~, ~)
            is_terminal = false;
        end

    end
end
