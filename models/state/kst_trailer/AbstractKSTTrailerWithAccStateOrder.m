classdef (Abstract) AbstractKSTTrailerWithAccStateOrder < AbstractStateOrder
    properties (Abstract, Constant)
        X uint32
        Y uint32
        V uint32 % velocity
        PSI uint32 % orientation
        HA uint32 % hitch angle
        DELTA uint32 % steering angle
        A uint32 % acceleration
    end

    properties (Constant)
        SIZE = 7
    end
end
