function inp = create_inp_for_a_delta_v(k, i, state, a, desired_delta, control_params)
    time_constant = control_params(1);
    delta_v = calc_steering_angle_velocity(state, k, desired_delta, time_constant);
    inp(i.DELTA_V, 1) = delta_v;
    inp(i.A, 1) = a;
end
