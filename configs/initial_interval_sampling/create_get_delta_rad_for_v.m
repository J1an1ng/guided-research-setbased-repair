function fun = create_get_delta_rad_for_v(constraints, min_delta_rad)

    function delta_rad = get_delta_rad(v)
        max_delta_rad = constraints.delta_max_for_v(v);
        delta_rad = 0.35 * max_delta_rad;
        delta_rad = min(max(delta_rad, min_delta_rad), max_delta_rad);
    end

    fun = @get_delta_rad;
end
