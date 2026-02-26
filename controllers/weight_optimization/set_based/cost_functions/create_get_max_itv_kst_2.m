function fun = create_get_max_itv_kst_2(k, constraints, delta_meas_dist)

    get_delta_range_for_v = create_get_delta_range_for_v(constraints, delta_meas_dist);

    function res = get_max_itv(c)
        res = zeros(k.get_size(), 2);

        res(k.X, :) = [-0.15, 0.15];
        res(k.Y, :) = [-0.13, 0.13];
        delta_rad = get_delta_range_for_v(c(k.V));
        res(k.DELTA, :) = c(k.DELTA) + [-delta_rad, delta_rad];
        res(k.V, :) = c(k.V) + [-0.7, 0.7];
        res(k.PSI, :) = [-0.013, 0.013];
        res(k.HA, :) = c(k.HA) + [-0.017, 0.017];

        res = interval(res(:, 1), res(:, 2));
    end

    fun = @get_max_itv;
end
