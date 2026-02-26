function inp_dist = zero_inp_dist_for_a_deltav(inp_order)
    arguments
        inp_order InpOrderADeltaV
    end

    inp_dist = zeros(inp_order.SIZE, 1);
    inp_dist(inp_order.A) = 0.;
    inp_dist(inp_order.DELTA_V) = 0.;
end
