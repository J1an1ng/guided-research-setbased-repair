function bounds = create_kst_trailer_inp_delta_state_bounds(state_order, params, drive_forward)
    if ~exist('drive_forward', 'var')
        drive_forward = 0;
    end

    k = state_order;

    kst_inp_delta_bounds = create_kst_inp_delta_state_bounds(k, params, drive_forward);
    state_min = kst_inp_delta_bounds.lb;
    state_max = kst_inp_delta_bounds.ub;

    state_min(k.HA) = -params.max_hitch_angle;
    state_max(k.HA) = params.max_hitch_angle;

    bounds = Bounds(state_min, state_max);
end
