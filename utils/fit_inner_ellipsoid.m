function Q = fit_inner_ellipsoid(points, c)
    n = size(points, 1);

    % Variables
    Q_inv = sdpvar(n, n, 'symmetric');  % Inverse of the ellipsoid shape matrix (must be symmetric)
    % Constraints: Q_inv must be positive definite
    constraints = [Q_inv >= 0];

    % Add the quadratic form constraints (p_i - c)' * Q_inv * (p_i - c) <= 1
    for i = 1:size(points, 2)
        constraints = [constraints, (points(:, i) - c)' * Q_inv * (points(:, i) - c) >= 1];
    end

    % Objective: maximize log(det(Q)), proportional to ellipsoid volume
    % as det(Q) = 1/det(Q_inv), we minimize log(det(Q_inv))
    objective = logdet(Q_inv);  % YALMIP minimizes, so we maximize by minimizing negative logdet

    % Set up the optimization problem using MOSEK
    options = sdpsettings('solver', 'mosek');
    sol = optimize(constraints, objective, options);

    % Display results
    if sol.problem == 0
        disp('Optimal center of ellipsoid:');
        disp('Optimal shape matrix Q:');
        Q = inv(value(Q_inv));
        disp(Q);
    else
        disp('Something went wrong!');
        sol.info;
        Q = [];
    end
end
