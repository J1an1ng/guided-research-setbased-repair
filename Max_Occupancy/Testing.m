% Initiate Turck Occupancy Computer
Truck_params = KSTTrailerParams1;
state_order = KSTTrailerStateOrderDefault();
occ_computer = TruckOccComputer(state_order, Truck_params);
state_bounds = create_kst_trailer_state_bounds(state_order, Truck_params);

% Define Threshold (when to reject occupancies that are too large)
lane_threshold = 4.0; % m

% Define initial search interval
lb = [-2; -0.25; -0.25]; % X Y Psi HA
ub = [2; 0.25; 0.25];
search_interval = interval(lb, ub);

abort_condition = 0;
enlargening_factor = 1.05;
iteration = 0;

%% Begin iteration
while abort_condition == 0
    iteration = iteration + 1;
    display(iteration);
    % Generate Random Points
    points = [randPoint(search_interval, 3000, 'uniform'), randPoint(search_interval, 1, 'extreme')];
    mirrored_points = mirror_on_point(points, search_interval.center);
    points = [points, mirrored_points];
    % calculate sampled intervals, their respective occupancies and the
    % distance of centroid to boundary of occupancy
    interval_samples = {};
    occupancy_samples = {};
    distances = {};
    accepted = [];
    rejected = [];
    decision = [];
    for kk = 1:length(points(1, :))
        % Calculate sampled interval
        sample_ub = abs(points(:, kk));
        sample_lb = -abs(points(:, kk));
        sample_ub = [0; sample_ub(1:state_order.DELTA - 2); 0; sample_ub(state_order.DELTA - 1:end)];
        sample_lb = [0; sample_lb(1:state_order.DELTA - 2); 0; sample_lb(state_order.DELTA - 1:end)];
        sample_ub = [sample_ub(1:state_order.V - 1); 0; sample_ub(state_order.V:end)];
        sample_lb = [sample_lb(1:state_order.V - 1); 0; sample_lb(state_order.V:end)];
        sample_interval = interval(sample_lb, sample_ub);
        interval_samples{1, end + 1} = sample_interval;

        % Calculate Occupancy
        sample_occ = occ_computer.compute_occupancy(zonotope(sample_interval));
        occupancy_samples{1, end + 1} = sample_occ;

        % Calculate Distance of center of trailer occupancy to boundary
        trailer_occ = sample_occ{2};
        [trailer_center_x, trailer_center_y] = trailer_occ.set.centroid;
        x_vertices = trailer_occ.set.Vertices(:, 1);
        y_vertices = trailer_occ.set.Vertices(:, 2);
        [d_min, varargout] = p_poly_dist([trailer_center_x], [trailer_center_y], x_vertices, y_vertices);
        distances{1, end + 1} = d_min;
        if d_min > lane_threshold
            decision(1, end + 1) = 0;
            rejected(1, end + 1) = kk;
        else
            decision(1, end + 1) = 1;
            accepted(1, end + 1) = kk;
        end
    end
    % extract accepted points

    which_accepted = logical(repmat(decision, length(points(:, 1)), 1));
    accepted_points = points(which_accepted);
    accepted_points = reshape(accepted_points, length(points(:, 1)), ...
        length(accepted_points) / length(points(:, 1)));

    % calculate enclosing polytope
    enclpoly = polytope.enclosePoints(accepted_points);
    % New search interval is enclosing polytope but enlarged a little bit
    new_search_interval = enlargening_factor * enclpoly;
    % if isa(search_interval, 'interval')
    %    search_interval = mptPolytope(search_interval.vertices');
    % end
    search_interval = new_search_interval;
    % search_interval = search_interval & new_search_interval;
    % search_interval = and(new_search_interval, search_interval);

    if length(accepted_points) / length(points) > 0.9950
        abort_condition = 1;
    end
    final_interval = enclpoly;

    %%
    % Plot what we have so far
    figure;
    tiledlayout('flow');
    xx = [2 3];
    states = ["Y", "PSI", "HA"];
    for zz = 1:size(xx, 2)
        nexttile;
        % plot(search_interval, [2, xx(1, zz)], 'EdgeColor', 'b');
        hold on;
        plot(points(1, :), points(xx(1, zz), :), '.', 'MarkerSize', 12);
        plot(accepted_points(1, :), accepted_points(xx(1, zz), :), '.', 'MarkerSize', 12, 'color', 'k');
        % plot(accepted_points(2,:), accepted_points(xx(zz),:), '*', 'MarkerSize', 12);
        plot(enclpoly, [1, xx(1, zz)], 'EdgeColor', 'k');
        xlabel('Y-Coordinate');
        ylabel(states(xx(zz)));
    end

end

figure;
tiledlayout('flow');
xx = [1 3];
lb_initial = [-0.1; -0.008; -0.008];
ub_initial = [0.1; 0.008; 0.008];
initial_interval = interval(lb_initial, ub_initial);
states = ["Y", "PSI", "HA"];
for zz = 1:size(xx, 2)
    nexttile;
    % plot(search_interval, [2, xx(1, zz)], 'EdgeColor', 'b');
    hold on;
    plot(initial_interval, [2, xx(1, zz)], 'EdgeColor', 'g', 'FaceColor', 'g');
    plot(enclpoly, [2, xx(1, zz)], 'EdgeColor', 'k');
    xlabel('PSI');
    ylabel(states(xx(zz)));
end

% plot(points(1,:), points(2,:), '.', 'MarkerSize',12);
% plot(mirrored_points(1,:), mirrored_points(2,:), '*', 'MarkerSize', 12);
% for ii = 1:length(interval_samples)
%    plot(interval_samples{ii});
% end

function mirrored = mirror_on_point(points, fix)
    mirrored = [];
    for kk = 1:length(points(1, :))
        distance_vector = points(:, kk) - fix;
        mirrored_point = fix - distance_vector;
        mirrored = [mirrored, mirrored_point];
    end
end
