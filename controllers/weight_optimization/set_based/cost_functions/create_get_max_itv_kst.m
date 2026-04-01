function fun = create_get_max_itv_kst(k, constraints)

    function res = get_max_itv(c)
        res = zeros(k.get_size(), 2);

        res(k.X, :) = [-0.05, 0.05];
        res(k.Y, :) = [-0.06, 0.06];
        % FIXME choose carefully...
        delta_rad = 0.3 * constraints.delta_max_for_v(c(k.V));
        res(k.DELTA, :) = c(k.DELTA) + [-delta_rad, delta_rad];
        res(k.V, :) = c(k.V) + [-0.5, 0.5];
        res(k.PSI, :) = [-0.001, 0.001];
        res(k.HA, :) = c(k.HA) + [-0.0015, 0.0015];

        res = interval(res(:, 1), res(:, 2));
    end

    fun = @get_max_itv;
end
