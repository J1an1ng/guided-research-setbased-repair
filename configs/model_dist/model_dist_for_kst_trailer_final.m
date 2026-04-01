function dist = model_dist_for_kst_trailer_final(state_order)
    arguments
        state_order KSTTrailerStateOrder
    end

    k = state_order;
    dist = model_dist_for_kst_final(k);

    dist(k.HA) = 0.0001;
end
