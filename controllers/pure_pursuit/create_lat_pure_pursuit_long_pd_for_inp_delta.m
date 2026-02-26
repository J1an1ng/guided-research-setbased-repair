function control_fun = create_lat_pure_pursuit_long_pd_for_inp_delta(reduced_kst_state_order, position_state_order)

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

        w_long = control_params(1);
        w_velo = control_params(2);

        epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);

        %% FIXME
        %% FIXME CC better without ref_inp(inp_order.A), as it is erroneous in the
        %% ref traj
        a = w_long * epsilon_long + w_velo * (v_ref - v); % + ref_inp(inp_order.A);

        desired_delta = lat_pure_pursuit(state, lookahead_ref, state_order, params);

        inp_creator = InpCreatorForADelta(state_order, inp_order, params);
        u = inp_creator.create_inp(state, a, desired_delta);
    end

    control_fun = @lat_pure_pursuit_long_pd;
end
