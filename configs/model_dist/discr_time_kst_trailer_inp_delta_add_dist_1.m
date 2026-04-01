function dist = discr_time_kst_trailer_inp_delta_add_dist_1(state_order)
    arguments
        state_order KSTTrailerInpDeltaStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    % From Lauras Paper
    dist(k.X) = 0.001; % FIXME
    dist(k.Y) = 0.001;

    dist(k.PSI) = 0.0001;

    % From Lauras Paper
    dist(k.V) = 0.001;

    dist(k.HA) = 0.0001;
end
