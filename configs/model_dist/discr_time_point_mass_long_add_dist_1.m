function dist = discr_time_point_mass_long_add_dist_1(state_order)
    arguments
        state_order PointMassLongStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    % From Lauras Paper
    dist(k.S) = 0.001; % FIXME

    % From Lauras Paper
    dist(k.V) = 0.001;
end
