function dx = kst_trailer_with_acc(x, u, state_order, inp_order, params)
    k = state_order;
    i = inp_order;

    delta = x(k.DELTA);
    v = x(k.V);
    psi = x(k.PSI);
    ha = x(k.HA);
    a = x(k.A);

    signum = sign(v);
    signum = (signum + 1) / 2;

    dx(k.X, 1) = v * cos(psi);
    dx(k.Y, 1) = v * sin(psi);
    dx(k.PSI, 1) = v * tan(delta) / params.truck_wheelbase;
    dx(k.HA, 1) = -v * (sin(ha) / params.trailer_wheelbase + tan(delta) / params.truck_wheelbase);
    dx(k.DELTA, 1) = u(i.DELTA_V);
    dx(k.V, 1) = signum * a; % sign for setting v' to 0 if v <= 0
    dx(k.A, 1) = u(i.J);
end
