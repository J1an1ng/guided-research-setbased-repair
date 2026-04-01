function bounds = create_kst_inp_delta_state_bounds(state_order, params, drive_forward)
    if ~exist('drive_forward', 'var')
        drive_forward = 0;
    end

    v_min = params.v_min;
    if drive_forward
        v_min = 0;
    end

    k = state_order;

    [state_min, state_max] = create_empty_bounds(state_order.get_size());

    state_min(k.V) = v_min;
    state_max(k.V) = params.v_max;

    bounds = Bounds(state_min, state_max);
end
