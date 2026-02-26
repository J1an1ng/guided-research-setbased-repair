function u = pd_controller_long_only(state, ref_state, ref_inp, state_order, inp_order, ~, weights)
    %% We do not use argument validation here for the sake of performance
    %% (see matlab_feature_testing/function_arg_with_validation_performance)

    k = state_order;

    %% Parse State
    % actual initial state (controlled)
    s = state(k.S);
    v = state(k.V);

    % reference state
    s_ref = ref_state(k.S);
    v_ref = ref_state(k.V);

    w_1 = weights(1);
    w_2 = weights(2);

    %% Errors:
    epsilon_long = s_ref - s;

    u(inp_order.A, 1) = w_1 * epsilon_long + w_2 * (v_ref - v) + ref_inp(inp_order.A);
end
