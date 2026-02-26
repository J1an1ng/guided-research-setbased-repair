function fun = prevent_backward_driving(model)
    fun = @(x, u, state_order, inp_order, params) ...
        inner_prevent_backward_driving(model, x, u, state_order, inp_order, params);
end

function dx = inner_prevent_backward_driving(model, x, u, ...
    state_order, inp_order, params)
    if x(state_order.V) < 0
        x(state_order.V) = 0;
    end

    dx = model(x, u, state_order, inp_order, params);

    THRESHOLD = 0.01;
    if x(state_order.V) <= THRESHOLD && dx(state_order.V) < 0
        dx(state_order.V) = 0;
    end
end
