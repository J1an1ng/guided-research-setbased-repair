classdef PrimitiveFragment < handle
    properties (SetAccess = immutable, GetAccess = public)
        is_terminal logical = false
        final_set SetContainer
    end

    methods

        function obj = PrimitiveFragment(is_terminal, final_set)
            obj.is_terminal = is_terminal;
            obj.final_set = final_set;
        end

    end
end
