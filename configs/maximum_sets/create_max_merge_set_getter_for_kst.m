function f = create_max_merge_set_getter_for_kst(k, constraints)

    function res = get_max_itv(c)
        res = zeros(k.SIZE, 2);

        res(k.X, :) = [-0.15, 0.15];
        res(k.Y, :) = [-0.09, 0.09];
        % delta is controlled very well, so there is no need here
        % to take a large interval; it would also lead to larger
        % sets in future
        delta_rad = 0.15 * constraints.delta_max_for_v(c(k.V));
        res(k.DELTA, :) = c(k.DELTA) + [-delta_rad, delta_rad];
        res(k.V, :) = c(k.V) + [-0.7, 0.7];
        res(k.PSI, :) = [-0.007, 0.007];
        res(k.HA, :) = c(k.HA) + [-0.016, 0.016];

        res = interval(res(:, 1), res(:, 2));
    end

    f = @get_max_itv;
end
