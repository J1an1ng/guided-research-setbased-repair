% parameter set of the kinematic single track model
% with on-axle trailer (truck model)
classdef VehicleParams
    properties (Constant)
        l = 5.100  % vehicle length [m]
        w = 2.550  % vehicle width [m]
        a = 1.8  % distance from spring mass center of gravity to front axle [m];
        b = 1.8  % distance from spring mass center of gravity to rear axle [m];
        l_wb = VehicleParams.a + VehicleParams.b

        % steering
        steering_min = -0.55  % minimum steering angle [rad]
        steering_max = 0.55  % maximum steering angle [rad]
        steering_v_min = -0.7103  % minimum steering velocity [rad/s]
        steering_v_max = 0.7103  % maximum steering velocity [rad/s]

        % longitudinal constraints
        v_min = -2.78  % minimum velocity [m/s]
        v_max = 22.22  % minimum velocity [m/s]
        v_switch = 7.824  % switching velocity [m/s]
        a_min = -6.8  % 11.5; %maximum absolute acceleration [m/s^2]
        a_max = 1.6  % 11.5; %maximum absolute acceleration [m/s^2]

        trailer_l = 13.6   % trailer length
        trailer_w = 2.55   % trailer width
        trailer_l_hitch = 12.00   % hitch length
        trailer_l_total = 16.5   % total system length
        trailer_l_wb = 8.1   % trailer wheelbase
    end
end
