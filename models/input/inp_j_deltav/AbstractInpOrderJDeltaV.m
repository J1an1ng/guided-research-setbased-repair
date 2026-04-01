classdef (Abstract) AbstractInpOrderJDeltaV < AbstractInpOrder
    properties (Abstract, Constant)
        J uint32
        DELTA_V uint32
    end

    properties (Constant)
        SIZE = 2
    end
end
