function u = pd_controller_3(state, ref_state, state_order, inp_order, ~, ...
    control_params)
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

    w_1 = control_params(1);
    w_2 = control_params(2);
    w_3 = control_params(3);
    w_4 = control_params(4);
    w_5 = control_params(5);

    %% Errors:
    epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);
    epsilon_lat = -sin(psi_ref) * (x_ref - x) + cos(psi_ref) * (y_ref - y);

    % We do not use this anymore as the derivative of dirac is bad for reachability analysis
    % (it produces infinity)
    % signum = sign(v);
    % signum = (signum + 1) / 2; % sign for setting v' to 0 if v <= 0
    v_non_zero = v + 1.0;

    u(inp_order.DELTA_V, 1) = w_1 * epsilon_lat + w_2 * (psi_ref - psi) + ...
        w_3 * (delta_ref - delta) / v_non_zero;
    u(inp_order.A, 1) = w_4 * epsilon_long + w_5 * (v_ref - v);
end
