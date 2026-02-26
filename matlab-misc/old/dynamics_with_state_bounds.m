function fun = dynamics_with_state_bounds(model, state_bounds)
    fun = @(x, u, state_order, inp_order, params) ...
        inner_dynamics_with_state_bounds(model, x, u, ...
        state_order, inp_order, params, state_bounds);
end

function dx = inner_dynamics_with_state_bounds(model, x, u, ...
    state_order, inp_order, params, state_bounds)

    dx = model(x, u, state_order, inp_order, params);

    dim_in_bounds = state_bounds.check_each_dim(x);
    dx_sign = sign(dx);
    set_dx_to_zero = max(dim_in_bounds * dx_sign, 0);
    dx(set_dx_to_zero) = 0;
end
