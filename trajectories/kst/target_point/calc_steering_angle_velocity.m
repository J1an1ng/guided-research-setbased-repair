function delta_v = calc_steering_angle_velocity(state, state_order, desired_delta, time_constant)
    delta = state(state_order.DELTA);
    delta_v = (desired_delta - delta) / time_constant;
    % delta_v_max = params.steering_v_max;
    % delta_v = delta_v_max * (1 - 2 / (exp(2 * delta_v) + 1));
end
