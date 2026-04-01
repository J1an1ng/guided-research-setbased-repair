function dist = model_dist_for_kst_trailer_1(state_order)
    arguments
        state_order KSTTrailerStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    % From Lauras Paper
    dist(k.X) = 0; % 0.03;
    dist(k.Y) = 0; % 0.02;

    dist(k.PSI) = 0;

    dist(k.DELTA) = 0;

    % From Lauras Paper
    dist(k.V) = 0; % 0.06;

    dist(k.HA) = 0; % 0.001;
end
