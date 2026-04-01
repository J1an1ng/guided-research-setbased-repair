function dx = discretely_controlled_model( ...
        state_collection, model_params, ...
        model, controller, ...
        state_order, inp_order, params, model_param_conf)

    k = state_order;
    state_sep1 = k.get_size();
    state_sep2 = state_sep1 + k.get_size();

    state = state_collection(1:state_sep1);
    measured_state = state_collection(state_sep1 + 1:state_sep2);

    [~, current_model_add_dist, current_inp_dist, ...
        ref_state, ref_inp, weights] = model_param_conf.split(model_params);

    u = controller(measured_state, ref_state, ref_inp, state_order, inp_order, params, weights);

    dx(1:state_sep1, 1) = model(state, u + current_inp_dist, state_order, ...
        inp_order, params) + current_model_add_dist;
    dx(state_sep1 + 1:state_sep2, 1) = zeros(k.get_size(), 1);
end
