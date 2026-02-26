function dx = point_mass_long(x, u, state_order, inp_order, ~)
    k = state_order;
    i = inp_order;

    v = x(k.V);

    dx(k.S, 1) = v;
    dx(k.V, 1) = u(i.A);
end
