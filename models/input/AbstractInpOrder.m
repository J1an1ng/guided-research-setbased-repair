classdef (Abstract) AbstractInpOrder < handle & matlab.mixin.Heterogeneous
    % Hint: inheriting from matlab.mixin.Heterogeneous prevents that we need to
    % define a default concrete class for this abstract class when using the
    % abstract class as type validator.
    properties (Abstract, Constant)
        SIZE uint32
    end
end
