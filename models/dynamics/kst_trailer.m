function dx = kst_trailer(x, u, state_order, inp_order, params)
    k = state_order;
    i = inp_order;

    delta = x(k.DELTA);
    v = x(k.V);
    psi = x(k.PSI);
    ha = x(k.HA);

    psi_dot = calc_psi_dot(v, delta, params.truck_wheelbase);

    dx(k.X, 1) = v * cos(psi);
    dx(k.Y, 1) = v * sin(psi);
    dx(k.DELTA, 1) = u(i.DELTA_V);
    dx(k.V, 1) = u(i.A);
    dx(k.PSI, 1) = psi_dot;
    dx(k.HA, 1) = calc_alpha_dot(v, ha, params.trailer_wheelbase, params.truck_dist_from_rear_axle_to_hitch, psi_dot);
end
