function state = kst_trailer_dt(state, u, dt, state_order, inp_order, params)
    tspan = [0 dt];
    fun = @(t, x) kst_trailer(x, u, state_order, inp_order, params);
    [~, states] = ode45(fun, tspan, state);
    state = states(end, :)';
end
