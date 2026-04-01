classdef PredefinedTraj < AbstractInpTraj
    properties (Constant)
        id_properties = []
    end

    methods
        function obj = PredefinedTraj(times, inputs, states, ...
            is_terminal, ext_horizon_states)
            obj.set_simulation_result(times, inputs, states, ...
                is_terminal, ext_horizon_states);
        end

        function inp_at(~, ~, ~, ~)
            %% FIXME: instead of having this class, it would be better to have a class that 
            %% holds the simulation result - this new class should then be used by e.g. the
            %% ControlledModelParamConfig class
            throw(MException('PredefinedTraj:inp_at', 'inp_at is not implemented for PredefinedTraj'));
        end
    end
    
end