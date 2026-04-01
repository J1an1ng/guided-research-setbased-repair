classdef KSTParams < handle & matlab.mixin.Heterogeneous
    %% Hint: we use a handle class for this rather than a struct,
    %% as we often pass the used instance of this class around as arguments
    %% (more precisely: the instance of its subclass). As a struct is passed
    %% by value in Matlab, using a handle-class is faster. See also speed test
    %% in matlab_feature_testing/function_arg_handle_vs_value

    properties (SetAccess = immutable, GetAccess = public)
        truck_length % vehicle length [m]
        truck_width % vehicle width [m]
        truck_wheelbase % distance between front and rear axles [m]
        truck_dist_from_rear_to_rear_axle % distance from front of vehicle to front axle [m]

        % friction constants
        a_min_total % positive; minimum deceleration (for friction circle) [m/s^2]
        get_a_min_lat % positive; minimum lateral deceleration [m/s^2]

        % state and input bounds
        steering_min
        steering_max
        steering_v_min
        steering_v_max

        v_min
        v_max

        a_min
        a_max

        max_road_width

        v_range
        max_steering_wheel_rate_range
    end

    properties (Dependent)
        l_front % distance from rear axle to front of vehicle [m]
        l_rear % distance from rear axle to rear of vehicle [m]
    end

    methods

        function l_front = get.l_front(obj)
            l_front = obj.truck_length - obj.truck_dist_from_rear_to_rear_axle;
        end

        function l_rear = get.l_rear(obj)
            l_rear = obj.truck_dist_from_rear_to_rear_axle;
        end

        function obj = KSTParams(params)
            obj.truck_length = params.truck_length;
            obj.truck_width = params.truck_width;
            obj.truck_wheelbase = params.truck_wheelbase;
            obj.truck_dist_from_rear_to_rear_axle = params.truck_dist_from_rear_to_rear_axle;

            obj.a_min_total = params.a_min_total;
            obj.get_a_min_lat = params.get_a_min_lat;

            obj.steering_min = params.steering_min;
            obj.steering_max = params.steering_max;
            obj.steering_v_min = params.steering_v_min;
            obj.steering_v_max = params.steering_v_max;

            obj.v_min = params.v_min;
            obj.v_max = params.v_max;

            obj.a_min = params.a_min;
            obj.a_max = params.a_max;

            obj.max_road_width = params.max_road_width;

            obj.v_range = params.v_range;
            obj.max_steering_wheel_rate_range = params.max_steering_wheel_rate_range;
        end

    end
end
