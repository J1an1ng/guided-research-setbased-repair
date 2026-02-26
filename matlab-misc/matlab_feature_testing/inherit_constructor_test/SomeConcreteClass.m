classdef SomeConcreteClass < SomeBaseClass
    properties (Constant)
        a = 3
    end

    properties
        y
    end

    methods

        function obj = SomeConcreteClass(y)
            obj.y = y;
        end

    end
end
