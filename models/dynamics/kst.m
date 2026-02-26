% KST - Kinematic Single-Track model
function dx = kst(x, u, state_order, inp_order, params)
    k = state_order;
    i = inp_order;

    delta = x(k.DELTA);
    v = x(k.V);
    psi = x(k.PSI);

    dx(k.X, 1) = v * cos(psi);
    dx(k.Y, 1) = v * sin(psi);
    dx(k.PSI, 1) = v * tan(delta) / params.truck_wheelbase;

    if isprop(i, 'DELTA')
        delta_cmd = u(i.DELTA);
        if isprop(params, 'steering_time_constant') && ~isempty(params.steering_time_constant)
            tau = params.steering_time_constant;
        else
            tau = 0.0;
        end
        tau = max(tau, 1.0e-6);
        dx(k.DELTA, 1) = (delta_cmd - delta) / tau;
    else
        dx(k.DELTA, 1) = u(i.DELTA_V);
    end

    dx(k.V, 1) = u(i.A);
end
