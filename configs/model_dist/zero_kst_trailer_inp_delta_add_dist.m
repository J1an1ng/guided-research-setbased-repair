function dist = zero_kst_trailer_inp_delta_add_dist(state_order)
    arguments
        state_order KSTTrailerInpDeltaStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    % From Lauras Paper
    dist(k.X) = 0;
    dist(k.Y) = 0;

    dist(k.PSI) = 0;

    % From Lauras Paper
    dist(k.V) = 0;

    dist(k.HA) = 0;
end
