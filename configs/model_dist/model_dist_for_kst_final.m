function dist = model_dist_for_kst_final(state_order)
    arguments
        state_order KSTStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    dist(k.X) = 0.0001;
    dist(k.Y) = 0.0001;

    dist(k.PSI) = 0.0001;

    dist(k.DELTA) = 0.0001;

    dist(k.V) = 0.0001;
end
