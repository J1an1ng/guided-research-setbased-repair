function dist = meas_dist_for_kst_final(state_order)
    arguments
        state_order KSTStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);

    % From: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8306879&tag=1
    % Lidar SLAM reaches precision of 0.017m in average for lateral position
    dist(k.X) = 0.03;
    dist(k.Y) = 0.01;

    % From: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
    % after talking to xx: could be even less than 0.5 degree...
    % In "A review of visual SLAM methods for autonomous driving vehicles":
    dist(k.PSI) = 0.0001; % 0.017; %0.001;

    % From: https://www.imc-tm.de/produkte/sensorik-loesungen/messlenkraeder
    % 0.045 deg precision
    % Here even better: https://www.abdynamics.com/en/blog/2018/
    % why-do-we-need-to-measure-steering-angle-to-0-001-degrees
    % Here: https://www.hella.com/bus/assets/media_global/1515_KI_TAS_HELLA_EN.pdf
    dist(k.DELTA) = 0.0001;

    % From MAN
    dist(k.V) = 0.03;
end
