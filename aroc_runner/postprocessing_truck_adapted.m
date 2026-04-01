function R = postprocessing_truck_adapted(R)
    l = VehicleParams.l;             % length of the truck
    w = VehicleParams.w;            % width of the truck
    l_wb = VehicleParams.l_wb;          % wheelbase of the truck
    l_t = VehicleParams.trailer_l;          % lenght of the trailer
    l_hitch = VehicleParams.trailer_l_hitch;        % hitch length trailer

    % settings for occupancy set computation
    dt = 0.05;
    order = 3;
    tol = 0.05;

    % compute occupancy set for the truck
    l1 = l_wb + (l - l_wb) / 2;
    l2 = (l - l_wb) / 2;

    truck = zonotope(interval([-l2; -w / 2], [l1; w / 2]));

    f = @(x, p) [x(5) + cos(x(2)) * p(1) - sin(x(2)) * p(2); ...
                 x(6) + cos(x(2)) * p(2) + sin(x(2)) * p(1)];

    R1 = postprocessingPolygon(f, truck, R, dt, order, tol);

    % compute occupancy set for the trailer
    dl = l_t - l_hitch;

    trailer = zonotope(interval([-l_hitch; -w / 2], [dl; w / 2]));

    f = @(x, p) [x(5) + cos(x(2) + x(3)) * p(1) - sin(x(2) + x(3)) * p(2); ...
                 x(6) + cos(x(2) + x(3)) * p(2) + sin(x(2) + x(3)) * p(1)];

    R2 = postprocessingPolygon(f, trailer, R, dt, order, tol);

    % compute occupancy sets for truck and trailer
    R = R1;

    for i = 1:length(R)
        R{i}.set = R1{i}.set | R2{i}.set;
    end
end
