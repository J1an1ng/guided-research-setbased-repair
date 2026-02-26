function u = pd_controller_2(state, ref_state, ref_inp, state_order, inp_order, params, ...
    weights)
    %% We do not use argument validation here for the sake of performance
    %% (see matlab_feature_testing/function_arg_with_validation_performance)

    k = state_order;

    %% Parse State
    % actual initial state (controlled)
    x = state(k.X);
    y = state(k.Y);
    delta = state(k.DELTA);
    v = state(k.V);
    psi = state(k.PSI);

    % reference state
    x_ref = ref_state(k.X);
    y_ref = ref_state(k.Y);
    delta_ref = ref_state(k.DELTA);
    v_ref = ref_state(k.V);
    psi_ref = ref_state(k.PSI);

    w_1 = weights(1);
    w_6 = weights(6);
    w_2 = weights(2);
    w_3 = weights(3);
    w_4 = weights(4);
    w_5 = weights(5);

    %% Errors:
    epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);
    epsilon_lat = -sin(psi_ref) * (x_ref - x) + cos(psi_ref) * (y_ref - y);
    epsilon_lat_dot = ...
        -(v_ref * tan(delta_ref) / params.truck_wheelbase) * epsilon_long ...
        - sin(psi_ref) * (v_ref * cos(psi_ref) - v * cos(psi)) ...
        + cos(psi_ref) * (v_ref * sin(psi_ref) - v * sin(psi));

    omega_ref = v_ref * tan(delta_ref) / params.truck_wheelbase;
    omega = v * tan(delta) / params.truck_wheelbase;

    u(inp_order.DELTA_V, 1) = w_1 * epsilon_lat + w_6 * epsilon_lat_dot + ...
        w_2 * (psi_ref - psi) + w_3 * (omega_ref - omega); % - w_4 * (delta_ref - delta));
    u(inp_order.A, 1) = w_4 * epsilon_long + w_5 * (v_ref - v);

    u = u + ref_inp;
end
