%% With this mixin, we do not need to define a default value when using
%% this abstract class as property validator (i.e., as type for a property in another class)
classdef (Abstract) MyAbstractClass < handle & matlab.mixin.Heterogeneous
    properties (Constant, Abstract)
        a double
        b uint32
    end
end
