function u = pd_controller_inp_delta_4d(state, ref_state, ref_inp, state_order, inp_order, params, weights)
    %% We do not use argument validation here for the sake of performance
    %% (see matlab_feature_testing/function_arg_with_validation_performance)

    k = state_order;

    %% Parse State
    x = state(k.X);
    y = state(k.Y);
    v = state(k.V);
    psi = state(k.PSI);

    x_ref = ref_state(k.X);
    y_ref = ref_state(k.Y);
    v_ref = ref_state(k.V);
    psi_ref = ref_state(k.PSI);

    w_1 = weights(1);
    w_2 = weights(2);
    w_4 = weights(4);
    w_5 = weights(5);

    %% Errors:
    [epsilon_long, epsilon_lat] = get_pos_deviation(psi_ref, x_ref, y_ref, x, y);

    u(inp_order.DELTA, 1) = w_1 * epsilon_lat + w_2 * (psi_ref - psi);
    u(inp_order.A, 1) = w_4 * epsilon_long + w_5 * (v_ref - v);

    u = u + ref_inp;
        u(inp_order.DELTA, 1) = scaled_tanh(u(inp_order.DELTA, 1), params.steering_max, -params.steering_min, 10);
        u(inp_order.A, 1) = scaled_tanh(u(inp_order.A, 1), params.a_max, -params.a_min, 10);

end
