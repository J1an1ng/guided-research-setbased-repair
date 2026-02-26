function f = create_total_max_set_getter_for_kst(k, constraints)

    function res = get_total_max_itv(c)
        res = zeros(k.SIZE, 2);
        res(k.X, :) = [-0.3, 0.3];
        res(k.Y, :) = [-0.18, 0.18];
        delta_rad = constraints.delta_max_for_v(c(k.V));
        res(k.DELTA, :) = c(k.DELTA) + [-delta_rad, delta_rad];
        res(k.V, :) = c(k.V) + [-1.5, 1.5];
        res(k.PSI, :) = [-0.01, 0.01];
        res(k.HA, :) = c(k.HA) + [-0.025, 0.025];

        res = interval(res(:, 1), res(:, 2));
    end

    f = @get_total_max_itv;
end
