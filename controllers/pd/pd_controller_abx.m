function u = pd_controller_abx(state, ref_state, ref_inp, state_order, inp_order, ~, weights)
    %% We do not use argument validation here for the sake of performance
    %% (see matlab_feature_testing/function_arg_with_validation_performance)

    k = state_order;

    %% Parse State
    % actual initial state (controlled)
    x = state(k.X);
    y = state(k.Y);
    v = state(k.V);
    psi = state(k.PSI);

    % reference state
    x_ref = ref_state(k.X);
    y_ref = ref_state(k.Y);
    v_ref = ref_state(k.V);
    psi_ref = ref_state(k.PSI);

    w_1 = weights(1);
    w_2 = weights(2);
    w_3 = weights(3);
    w_4 = weights(4);

    %% Errors:
    [epsilon_long, epsilon_lat] = get_pos_deviation(psi_ref, x_ref, y_ref, x, y);

    u(inp_order.DELTA, 1) = w_1 * epsilon_lat + w_2 * (psi_ref - psi) + ref_inp(inp_order.DELTA);
    u(inp_order.A, 1) = w_3 * epsilon_long + w_4 * (v_ref - v); % + ref_inp(inp_order.A); %% FIXME implement like this on Autobox

    % This would prevent positive acceleration, but directly leads to reachset explosion:
    % -log(1 + exp(-new_a));

    %% On the abx controller, the ref input is only added for the curvature request
    % u = u + ref_inp;

    % if isnumeric(u)
    %     if u(inp_order.A) < -10.0 || u(inp_order.A) > 5.0
    %         warning('Acceleration out of bounds: %f', u(inp_order.A));
    %     end
    % end

    % With the next line, the computation of the derivative gets stuck already fml
    % u(inp_order.A) = scaled_tanh(u(inp_order.A), params.a_max, -params.a_min, 10);
end
