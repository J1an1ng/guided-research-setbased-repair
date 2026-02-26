classdef PredefinedInputTraj < AbstractInpTraj
    properties (Constant)
        id_properties = []
    end

    properties (SetAccess = private, GetAccess = public)
        predefined_inputs (:, :) double
        time_per_input double
    end

    methods

        function obj = PredefinedInputTraj(predefined_inputs, time_per_input)
            obj.predefined_inputs = predefined_inputs;
            obj.time_per_input = time_per_input;
        end

        function inp = inp_at(obj, ~, t, ~)
            idx = floor(t / obj.time_per_input) + 1;
            inp = obj.predefined_inputs(:, idx);
        end

    end

end
