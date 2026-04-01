function fun = create_get_max_rads(k, red_k, constraints, delta_meas_dist)
    get_delta_rad = create_get_delta_rad_for_v(constraints, delta_meas_dist);

    % Hint: these values should be rather small; we do not enforce them anyway
    % but just take them as an additional possibility for sets to be merged
    max_v_rad = 0.7;
    max_ha_rad = 0.025;

    function res = get_max_rads(c)
        res = zeros(red_k.SIZE, 1);
        res(red_k.DELTA) = get_delta_rad(c(k.V));
        res(red_k.V) = max_v_rad;
        res(red_k.HA) = max_ha_rad;
    end

    fun = @get_max_rads;
end
