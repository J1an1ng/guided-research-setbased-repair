classdef SomeDataClass < handle
    properties
        a
        b
        c
    end

    methods

        function obj = SomeDataClass(a, b, c)
            obj.a = a;
            obj.b = b;
            obj.c = c;
        end

    end
end
