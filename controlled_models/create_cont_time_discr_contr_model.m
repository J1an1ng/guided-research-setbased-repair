function [system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, controller, ...
    state_order, inp_order, ...
    params, model_param_conf, state_proc, check_input_fun)
    arguments
        model
        controller
        state_order AbstractStateOrder
        inp_order AbstractInpOrder
        params
        model_param_conf ControlledModelParamConfig
        state_proc StateProcessor
        check_input_fun function_handle = @(x, u) zeros(length(u), 1)
    end

    function dx = get_system_dynamics(state_collection, model_params)
        [state, measured_state] = state_proc.split_and_process(state_collection);

        [~, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = model_param_conf.split(model_params);

        u = controller(measured_state, ref_state, ref_inp, state_order, inp_order, params, weights);

        bound_violations = check_input_fun(state, u);
        if any(bound_violations ~= 0)
            throw(InputOutOfBoundsException(bound_violations));
        end

        derivative_state = model(state, u + current_inp_dist, state_order, inp_order, params);
        dx = state_proc.create_derivative(state, derivative_state, current_model_add_dist);
    end

    system_dynamics = @get_system_dynamics;

    function output = get_system_output(state_collection, ~)
        [state, ~] = state_proc.split_and_process(state_collection);
        % Hint: when using measured_state as output (which is always
        % the one at the beginning of the time interval), we end up
        % with constant reachsets
        output = state;
    end

    system_output = @get_system_output;
end
