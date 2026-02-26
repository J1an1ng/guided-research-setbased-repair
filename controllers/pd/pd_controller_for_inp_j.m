function u = pd_controller_for_inp_j(state, ref_state, state_order, inp_order, params)
    k = state_order;

    %% Parse State
    % actual state (controlled)
    x = state(k.X);
    y = state(k.Y);
    delta = state(k.DELTA);
    v = state(k.V);
    Psi = state(k.PSI);
    acc = state(k.A);
    % reference state
    xr = ref_state(k.X);
    yr = ref_state(k.Y);
    deltar = ref_state(k.DELTA);
    vr = ref_state(k.V);
    Psir = ref_state(k.PSI);
    accr = ref_state(k.A);

    %% Compute Control Inputs
    d_v_scale = 1; % Making this factor velocity dependent apparently
    % leads to reachset explosions
    w(1) = 8;   % weight for lateral error
    w(2) = 25; % weight for orientation error
    w(3) = 7; % weight for omega error
    w(7) = 0; % weight for keeping steering minimal

    j_scale = 1;
    w(4) = 16;  % weight for longitudinal error
    w(5) = 50;  % weight for velocity error
    w(6) = 20;  % weight for acceleration error

    %% Errors:
    epsilon_long = cos(Psir) * (xr - x) + sin(Psir) * (yr - y);
    epsilon_lat = -sin(Psir) * (xr - x) + cos(Psir) * (yr - y);

    omegar = vr * tan(deltar) / params.truck_wheelbase;
    omega = v * tan(delta) / params.truck_wheelbase;

    %% Test Acceleration and Steering Angle Bound
    % Consider uper acceleration bound;
    % FIXME: using this mechanism implies that j=0 whenever acc>=2.
    % Then, however, braking is not possible, as soon as the maximum
    % acceleration is reached. The same holds for the steering angle...
    signum_acc = sign(2 - acc);
    signum_acc = (signum_acc + 1) / 2;
    % Consider upper steering angle bound;
    signum_dv1 = sign(0.5 - delta);
    signum_dv1 = (signum_dv1 + 1) / 2;
    % Consider lower steering angle bound;
    signum_dv2 = sign(delta + 0.5);
    signum_dv2 = (signum_dv2 + 1) / 2;
    % Cosnider complete steering angle bound;
    signum_dv = signum_dv1 * signum_dv2;

    u(inp_order.DELTA_V, 1) = signum_dv * d_v_scale * ...
        (w(1) * epsilon_lat + w(2) * (Psir - Psi) + w(3) * ...
        (omegar - omega) - w(7) * (deltar - delta));
    u(inp_order.J, 1) = signum_acc * j_scale * (w(4) * epsilon_long + ...
        w(5) * (vr - v) + w(6) * (accr - acc));
end
