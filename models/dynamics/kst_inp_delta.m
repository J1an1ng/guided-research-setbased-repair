%% delta作为input 
% KST - Kinematic Single-Track model with steering angle as input
function dx = kst_inp_delta(x, u, state_order, inp_order, params)
    k = state_order;
    i = inp_order;

    v = x(k.V);
    psi = x(k.PSI);
    delta = u(i.DELTA);

    dx(k.X, 1) = v * cos(psi);
    dx(k.Y, 1) = v * sin(psi);
    dx(k.PSI, 1) = v * tan(delta) / params.truck_wheelbase;
    dx(k.V, 1) = u(i.A);
end
