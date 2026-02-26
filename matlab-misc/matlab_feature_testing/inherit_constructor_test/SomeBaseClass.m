classdef (Abstract) SomeBaseClass < handle
    properties (Abstract, Constant)
        a
    end

    properties (SetAccess = private)
        x
    end

    methods

        function obj = SomeBaseClass()
            obj.x = 10 * obj.a;
        end

    end
end
