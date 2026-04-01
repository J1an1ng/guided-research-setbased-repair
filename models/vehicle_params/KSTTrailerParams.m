classdef KSTTrailerParams < KSTParams
    %% Hint: we use a handle class for this rather than a struct,
    %% as we often pass the used instance of this class around as arguments
    %% (more precisely: the instance of its subclass). As a struct is passed
    %% by value in Matlab, using a handle-class is faster. See also speed test
    %% in matlab_feature_testing/function_arg_handle_vs_value

    properties (SetAccess = immutable, GetAccess = public)
        trailer_length % trailer length
        trailer_width % trailer width
        trailer_dist_from_front_to_hitch
        truck_dist_from_rear_axle_to_hitch
        trailer_wheelbase % trailer wheelbase

        max_hitch_angle

        a_min_lat_trailer
    end

    properties (Dependent)
        trailer_dist_from_rear_to_rear_axle % distance from rear of trailer to rear axle
    end

    methods

        function val = get.trailer_dist_from_rear_to_rear_axle(obj)
            val = obj.trailer_length - obj.trailer_wheelbase - obj.trailer_dist_from_front_to_hitch;
        end

    end

    methods

        function obj = KSTTrailerParams(params)
            obj = obj@KSTParams(params);

            obj.trailer_length = params.trailer_length;
            obj.trailer_width = params.trailer_width;
            obj.trailer_dist_from_front_to_hitch = params.trailer_dist_from_front_to_hitch;
            obj.truck_dist_from_rear_axle_to_hitch = params.truck_dist_from_rear_axle_to_hitch;
            obj.trailer_wheelbase = params.trailer_wheelbase;

            obj.max_hitch_angle = params.max_hitch_angle;
            obj.a_min_lat_trailer = params.a_min_lat_trailer;
        end

    end
end
