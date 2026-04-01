function bounds = create_inp_bounds_for_a_delta(i, params)
    [inp_min, inp_max] = create_empty_bounds(i.SIZE);

    inp_min(i.DELTA) = params.steering_min;
    inp_max(i.DELTA) = params.steering_max;

    inp_min(i.A) = params.a_min;
    inp_max(i.A) = params.a_max;

    bounds = Bounds(inp_min, inp_max);
end
