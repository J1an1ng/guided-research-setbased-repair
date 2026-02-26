function inp_dist = inp_dist_for_j_deltav_1(inp_order)
    arguments
        inp_order InpOrderJDeltaV
    end

    inp_dist = zeros(inp_order.SIZE, 1);
    % This is smaller than the one of Anka's Master thesis, as we
    % only consider braking, where the disturbance should not be not
    % that strong
    inp_dist(inp_order.J) = 0.05;
    inp_dist(inp_order.DELTA_V) = 0.04;
end
