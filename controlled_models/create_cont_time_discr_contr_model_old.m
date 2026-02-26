function [system_dynamics, system_output] = create_cont_time_discr_contr_model( ...
    model, controller, ...
    state_order, inp_order, ...
    params, model_param_conf, check_input_fun)

    if ~exist('check_input_fun', 'var') || isempty(check_input_fun)
        check_input_fun = @(x, u) zeros(length(u), 1);
    end

    k = state_order;
    state_sep1 = k.get_size();
    state_sep2 = state_sep1 + k.get_size();

    function dx = get_system_dynamics(state_collection, model_params)
        state = state_collection(1:state_sep1);
        measured_state = state_collection(state_sep1 + 1:state_sep2);

        [~, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = model_param_conf.split(model_params);

        u = controller(measured_state, ref_state, ref_inp, state_order, inp_order, params, weights);

        bound_violations = check_input_fun(state, u);
        if any(bound_violations ~= 0)
            throw(InputOutOfBoundsException(bound_violations));
        end

        dx(1:state_sep1, 1) = model(state, u + current_inp_dist, state_order, ...
            inp_order, params) + current_model_add_dist;
        dx(state_sep1 + 1:state_sep2, 1) = zeros(k.get_size(), 1);
    end

    system_dynamics = @get_system_dynamics;

    function output = get_system_output(state_collection, ~, ~)
        k = state_order;
        state_sep1 = k.get_size();
        state_sep2 = state_sep1 + k.get_size();

        output = state_collection(state_sep1 + 1:state_sep2);
        % output = state_collection;
    end

    system_output = @get_system_output;
end
