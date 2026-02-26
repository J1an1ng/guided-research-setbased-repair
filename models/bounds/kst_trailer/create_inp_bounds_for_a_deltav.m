function bounds = create_inp_bounds_for_a_deltav(i, params)
    [inp_min, inp_max] = create_empty_bounds(i.SIZE);

    factor_delta_v = 5.0;

    inp_min(i.DELTA_V) = factor_delta_v * params.steering_v_min;
    inp_max(i.DELTA_V) = factor_delta_v * params.steering_v_max;

    %% for ignoring acceleration bounds (when only optimizing lateral control)
    inp_min(i.A) = 4.0 * params.a_min;
    inp_max(i.A) = 4.0 * params.a_max;

    bounds = Bounds(inp_min, inp_max);
end
