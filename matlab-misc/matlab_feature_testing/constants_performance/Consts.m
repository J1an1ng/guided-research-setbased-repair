classdef Consts
    properties (Constant)
        A = 0.5
        B = 3
    end

    properties
        variable_a
        variable_b
    end

    methods

        function obj = Consts()
            obj.variable_a = 0.5;
            obj.variable_b = 3;
        end

    end
end
