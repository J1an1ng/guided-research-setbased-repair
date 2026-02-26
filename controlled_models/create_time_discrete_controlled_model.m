function [system_dynamics, system_output] = create_time_discrete_controlled_model(dt, k)

    function next_state = get_system_dynamics( ...
            state, model_params, ...
            model, controller, ...
            state_order, inp_order, params, model_param_conf, check_input_fun)

        %% FIXME use new infrastructure for this

        psi = state(state_order.PSI);
        cos_psi = cos(psi);
        sin_psi = sin(psi);

        [meas_dist, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = model_param_conf.split(model_params);

        m_x = meas_dist(k.X);
        m_y = meas_dist(k.Y);

        meas_dist(k.X) = cos_psi * m_x + -sin_psi * m_y;
        meas_dist(k.Y) = sin_psi * m_x + cos_psi * m_y;

        d_x = current_model_add_dist(k.X);
        d_y = current_model_add_dist(k.Y);

        current_model_add_dist(k.X) = cos_psi * d_x + -sin_psi * d_y;
        current_model_add_dist(k.Y) = sin_psi * d_x + cos_psi * d_y;

        u = controller(state + meas_dist, ref_state, ref_inp, state_order, inp_order, params, weights);

        bound_violations = check_input_fun(state, u);
        if any(bound_violations ~= 0)
            throw(InputOutOfBoundsException(bound_violations));
        end

        dx(1:state_order.get_size(), 1) = model(state, u + current_inp_dist, state_order, ...
            inp_order, params);

        % we use Euler method for integrating the ODE
        next_state = state + dt * dx + current_model_add_dist;

        % 2nd order Runge-Kutta (Heun's method) for more accurate integration
        % k1 = dx;
        % state_mid = state + (dt/2) * k1;
        % k2 = model(state_mid, u + current_inp_dist, state_order, inp_order, params);
        % next_state = state + dt * (0.5*k1 + 0.5*k2) + current_model_add_dist;
    end

    system_dynamics = @get_system_dynamics;

    function output = get_system_output(state, model_params, model_param_conf)
        meas_dist = model_param_conf.get_current_meas_dist(model_params);
        % meas_dist(4) = 0;

        psi = state(k.PSI);
        cos_psi = cos(psi);
        sin_psi = sin(psi);
        m_x = meas_dist(k.X);
        m_y = meas_dist(k.Y);

        meas_dist(k.X) = cos_psi * m_x + -sin_psi * m_y;
        meas_dist(k.Y) = sin_psi * m_x + cos_psi * m_y;

        output = state + meas_dist;
        output = output(1:4); %% FIXME: hack to remove hitch angle from output
    end

    system_output = @get_system_output;
end
