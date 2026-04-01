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
    dx(k.DELTA, 1) = u(i.DELTA_V);
    dx(k.V, 1) = u(i.A);
end


