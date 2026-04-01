function desired_delta = lat_pure_pursuit(state, ref_position, state_order, params)
    k = state_order;

    pos = state([k.X k.Y]);
    psi = state(k.PSI);

    pos_diff = ref_position - pos;
    % distance to ref point
    l_d = sqrt(pos_diff(1)^2 + pos_diff(2)^2);

    % Eliminate chained trigonometric functions by using direct computation
    % Instead of: alpha = atan2(pos_diff(2), pos_diff(1)) - psi
    % and then: sin(alpha)
    % We compute: sin(alpha) = (pos_diff(2) * cos(psi) - pos_diff(1) * sin(psi)) / l_d
    sin_alpha = (pos_diff(2) * cos(psi) - pos_diff(1) * sin(psi)) / l_d;

    desired_delta = atan(2 * params.truck_wheelbase * sin_alpha / l_d);
    % desired_delta = 2 * params.truck_wheelbase * sin_alpha / l_d;
end
