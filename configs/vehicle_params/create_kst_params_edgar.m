function res = create_kst_params_edgar()
    % Data are mainly from: https://www.volkswagen-nutzfahrzeuge.de/de/modelle/multivan/technische-daten.html
    % Some are just guessed

    params.truck_length = 4.973;
    params.truck_width = 1.941;
    params.truck_wheelbase = 3.124;
    params.truck_dist_from_rear_to_rear_axle = 0.9245;

    params.v_min = 0;
    %% TODO change if needed
    params.v_max = 100.0 / 3.6; % 100 km/h

    % This value arises from the maximum lateral acceleration of 3.0
    % and a maximum longitudinal acceleration of 4.5: \sqrt{3.0^2 + 4.5^2} = 5.4 with
    % some margin.
    params.a_min_total = 5.5;

    a_min_lat_low = 2.0;
    a_min_lat_high = 3.0;
    factor = (a_min_lat_low - a_min_lat_high) / (params.v_max - params.v_min);
    params.get_a_min_lat = @(v) factor * v + a_min_lat_high;

    % Computed from "Wendekreis"
    params.steering_min = -0.54;
    params.steering_max = 0.54;
    params.steering_v_min = -0.122;
    params.steering_v_max = 0.122;

    params.a_min = -4.5; % is actually even lower
    params.a_max = 2.5;

    params.max_road_width = 4.0;

    params.v_range = [];
    params.max_steering_wheel_rate_range = [];

    res = KSTParams(params);
end
