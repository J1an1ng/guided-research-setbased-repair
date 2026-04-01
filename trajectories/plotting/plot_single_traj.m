function res = plot_single_traj(traj, c, k, vehicle_plotter)
    res = {};
    if isprop(traj, "target_pos")
        p1 = scatter(traj.target_pos(1), traj.target_pos(2), 20, c, "filled");
        res = {p1};
    end
    X = traj.states(k.X, :);
    Y = traj.states(k.Y, :);
    p2 = plot(X, Y, 'color', c, 'LineWidth', 1);
    p3 = vehicle_plotter.plot_vehicle(traj.states(:, end), c);
    res = [res; {p2}; p3];
end
