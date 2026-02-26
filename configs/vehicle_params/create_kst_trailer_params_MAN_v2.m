function res = create_kst_trailer_params_MAN_v2()
    params.truck_length = 5.72;
    params.truck_width = 2.55;
    params.truck_wheelbase = 3.6;
    params.truck_dist_from_rear_to_rear_axle = 1.37;

    params.v_min = 0;
    params.v_max = 22.22;

    % This value arises from the maximum lateral acceleration of 2.0
    % and a maximum longitudinal acceleration of 3.0: \sqrt{2.0^2 + 3.0^2} = 3.6 with
    % some margin.
    params.a_min_total = 4.9;

    % This is not mentioned in the paper for the sake of brevity;
    % only the maximum overall acceleration is mentioned.
    a_min_lat_low = 1.6;
    a_min_lat_high = 2.0;
    factor = (a_min_lat_low - a_min_lat_high) / (params.v_max - params.v_min);
    params.get_a_min_lat = @(v) factor * v + a_min_lat_high;

    params.steering_min = -0.5;
    params.steering_max = 0.5;
    % maximum steering rate (but could be further restricted
    % depending on velocity)
    % Computed from Nope's maximum curvature rate: 0.0344
    params.steering_v_min = -0.122;
    params.steering_v_max = 0.122;

    % Mainly used for controller optimization
    params.a_min = -4.0;
    params.a_max = 1.6;

    params.max_road_width = 4.0;

    % from Christian; this is not mentioned in my paper for the sake of brevity
    params.v_range = [0 10 15 20 30 40 50 65 80 100];
    params.max_steering_wheel_rate_range = [2.5 2.5 2.077 1.379 1.03 0.856 0.489 0.367 0.245 0.245];

    params.trailer_length = 12.45;
    params.trailer_width = 2.55;
    params.trailer_dist_from_front_to_hitch = 1.17;
    params.truck_dist_from_rear_axle_to_hitch = 0.45;
    params.trailer_wheelbase = 7.8;

    params.max_hitch_angle = 0.26;

    params.a_min_lat_trailer = 2.0; % 1.5;

    res = KSTTrailerParams(params);
end
