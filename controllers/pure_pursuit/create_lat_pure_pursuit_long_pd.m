function control_fun = create_lat_pure_pursuit_long_pd(reduced_kst_state_order, position_state_order)

    function u = lat_pure_pursuit_long_pd(state, ref_state, ref_inp, state_order, inp_order, ...
            params, control_params)
        k = state_order;

        x = state(k.X);
        y = state(k.Y);
        v = state(k.V);

        % reference element
        x_ref = ref_state(reduced_kst_state_order.X);
        y_ref = ref_state(reduced_kst_state_order.Y);
        v_ref = ref_state(reduced_kst_state_order.V);
        psi_ref = ref_state(reduced_kst_state_order.PSI);

        ref_state = ref_state(reduced_kst_state_order.get_size() + 1:end);

        lookahead_x_ref = ref_state(position_state_order.X);
        lookahead_y_ref = ref_state(position_state_order.Y);
        lookahead_ref = [lookahead_x_ref; lookahead_y_ref];

        time_constant = control_params(1);
        w_long = control_params(2);
        w_velo = control_params(3);

        epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);

        a = w_long * epsilon_long + w_velo * (v_ref - v) + ref_inp(inp_order.A);

        desired_delta = lat_pure_pursuit(state, lookahead_ref, state_order, params);

        delta_v = calc_steering_angle_velocity(state, k, desired_delta, time_constant, params);
        u(inp_order.DELTA_V, 1) = delta_v;
        u(inp_order.A, 1) = a;
    end

    control_fun = @lat_pure_pursuit_long_pd;
end
