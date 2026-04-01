classdef SubTest < AbstractTest
    properties (Constant)
        x = 0
    end

    methods

        function res = test(obj, other_obj)
            o = other_obj; % should not make a copy
            res = o.x == obj.x;
            % res = other_obj.x == obj.x;
        end

    end
end
