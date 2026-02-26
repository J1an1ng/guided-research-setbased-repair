classdef AbstractKSTStateOrder < AbstractStateOrder
    % State order for kinematic single track model
    properties (Abstract, SetAccess = immutable, GetAccess = public)
        X uint32
        Y uint32
        V uint32 % velocity
        PSI uint32 % orientation
    end
end
