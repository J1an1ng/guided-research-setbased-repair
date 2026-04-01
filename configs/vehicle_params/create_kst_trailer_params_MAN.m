function res = create_kst_trailer_params_MAN()
    params.truck_length = 5.100;
    params.truck_width = 2.550;
    params.truck_wheelbase = 3.6;
    params.truck_dist_from_rear_to_rear_axle = 0.5;

    params.v_min = 0;
    params.v_max = 22.22;

    params.a_min_total = 6.8;  % assuming \mu=0.7
    % Value is from Nope; the vehicle can do more but
    % the current controller is not able to handle it

    a_min_lat_low = 1.0;
    a_min_lat_high = 1.5;

    factor = (a_min_lat_low - a_min_lat_high) / (params.v_max - params.v_min);
    params.get_a_min_lat = @(v) factor * v + a_min_lat_high;

    params.steering_min = -0.5;
    params.steering_max = 0.5;
    % maximum steering rate (but could be further restricted
    % depending on velocity)
    % Computed from Nope's maximum curvature rate: 0.0344
    params.steering_v_min = -0.122;
    params.steering_v_max = 0.122;

    params.a_min = -4.0;
    params.a_max = 1.6;

    params.max_road_width = 4.0;

    % from Christian
    params.v_range = [0 10 15 20 30 40 50 65 80 100];
    params.max_steering_wheel_rate_range = [2.5 2.5 2.077 1.379 1.03 0.856 0.489 0.367 0.245 0.245];

    params.trailer_length = 13.6;
    params.trailer_width = 2.55;
    params.trailer_dist_from_front_to_hitch = 0.9;
    params.truck_dist_from_rear_axle_to_hitch = 0.45;
    params.trailer_wheelbase = 7.8;

    params.max_hitch_angle = 0.26;

    params.a_min_lat_trailer = 5.0;

    res = KSTTrailerParams(params);
end
