function [system_dynamics, system_output] = create_cont_time_discr_contr_model_with_obs( ...
    model, controller, ...
    state_order, inp_order, ...
    params, model_param_conf, ...
    state_proc, check_input_fun)

    k = state_order;
    state_sep1 = k.get_size();
    state_sep2 = state_sep1 + k.get_size();
    state_sep3 = state_sep2 + k.get_size();

    function dx = get_system_dynamics(state_collection, model_params)
        state = state_collection(1:state_sep1);
        init_obs_state = state_collection(state_sep1 + 1:state_sep2);
        propagated_obs_state = state_collection(state_sep2 + 1:state_sep3);

        [~, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = model_param_conf.split(model_params);

        u = controller(init_obs_state, ref_state, ref_inp, state_order, inp_order, params, weights);

        bound_violations = check_input_fun(state, u);
        if any(bound_violations ~= 0)
            throw(InputOutOfBoundsException(bound_violations));
        end

        dx(1:state_sep1, 1) = model(state, u + current_inp_dist, state_order, ...
            inp_order, params) + current_model_add_dist;
        dx(state_sep1 + 1:state_sep2, 1) = zeros(k.get_size(), 1);
        dx(state_sep2 + 1:state_sep3, 1) = model(propagated_obs_state, u, ...
            state_order, inp_order, params);
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
