function fun = create_size_checker(k, constraints, delta_meas_dist, max_file_path, var_name, use_itv_approx)
    red_k = TrailerOccStateOrder();
    relevant_dims = red_k.create_state_order_mapping(k);
    max_set = load(max_file_path);
    max_poly = max_set.(var_name);

    if use_itv_approx
        max_poly = interval(max_poly);
    end

    get_delta_rad_for_v = create_get_delta_rad_for_v(constraints, delta_meas_dist);
    v_rad = 0.7;

    function res = check_size(some_set)
        res = false;

        some_set_relevant_dims = project(some_set, relevant_dims);
        some_set_relevant_dims = some_set_relevant_dims - center(some_set_relevant_dims);
        if ~contains(max_poly, some_set_relevant_dims)
            return
        end

        v_set = interval(project(some_set, k.V));
        if rad(v_set) > v_rad
            return
        end

        delta_set = interval(project(some_set, k.DELTA));
        delta_rad = get_delta_rad_for_v(center(v_set));
        if rad(delta_set) > delta_rad
            return
        end

        res = true;
    end

    fun = @check_size;
end
