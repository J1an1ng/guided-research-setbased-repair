classdef ClassWithDepProp < handle
    properties
        x = 0
    end

    properties (Dependent)
        y
    end

    methods

        function v = get.y(obj)
            v = obj.x;
            for i = 1:100000000
                v = v + log(i);
            end
        end

    end
end
