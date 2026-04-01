classdef Controller_Optimizer
    % CONTROLLER_OPTIMIZER For optimizing controller weights via fmincon
    %   ToDo
    %  Takes as 'controller'-input a control function
    %  u=controller(state, ref_state, weights);

    properties (Access = private)
        num_weights integer
        controller function_handle
        model function_handle
        initial_weights numeric
        disturbance Disturbances
    end

    methods

        function obj = Controller_Optimizer(controller, num_weights, distubrances)
            obj.controller = controller;
            obj.num_weights = num_weights;
            obj.initial_weights = first_guess;
            obj.disturbance = disturbances;
        end

        function controlled_model = build_model(controller, model)

        end

        function optimal_weights = optimize(obj, first_guess)
            % METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            optimal_weights = fmincon(obj.controller, first_guess);
        end

    end
end
