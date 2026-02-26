function res = create_kst_trailer_params_2()
    params.l = 5.100;
    params.w = 2.550;
    params.a = 1.8;
    params.b = 1.8;

    params.a_min_total = 6.8;  % assuming \mu=0.7
    % TODO could theoretically be more but the current
    % controller is not able to handle it
    params.a_min_lat = 3.0;

    params.steering_min = -0.26;
    params.steering_max = 0.26;
    % maximum steering rate (but could be further restricted
    % depending on velocity)
    params.steering_v_min = -0.7103;
    params.steering_v_max = 0.7103;

    params.v_min = 0;
    params.v_max = 22.22;

    params.a_min = -6.0;
    params.a_max = 1.6;

    params.max_road_width = 4.0;

    % from Christian
    params.v_range = [0 10 15 20 30 40 50 65 80 100];
    params.max_steering_wheel_rate_range = [2.5 2.5 2.077 1.379 1.03 0.856 0.489 0.367 0.245 0.245];

    params.trailer_l = 13.6;
    params.trailer_w = 2.55;
    params.trailer_l_hitch = 12.00;
    params.trailer_l_total = 16.5;
    params.trailer_l_wb = 8.1;

    params.max_hitch_angle = 0.26;

    params.a_min_lat_trailer = 7.8;

    res = KSTTrailerParams(params);
