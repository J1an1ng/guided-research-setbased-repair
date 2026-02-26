function bounds = create_inp_bounds_for_a(i, params)
    [inp_min, inp_max] = create_empty_bounds(i.SIZE);

    factor_1 = 3.0;
    factor_2 = 3.0;

    inp_min(i.A) = factor_1 * params.a_min;
    inp_max(i.A) = factor_2 * params.a_max;

    bounds = Bounds(inp_min, inp_max);
end
