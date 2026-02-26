function res = create_test_suite_from_data(rec_data, dt, dyn_class, length_test_case, margin_end, k, inp_order, ext_horizon_time, dyn_conf, dyn_conf_values)
    %% FIXME check which dim
    num_test_cases = floor((size(rec_data.output.x, 2) - margin_end) / length_test_case);

    res = cell(num_test_cases, 1);

    % we need at least 1 more state for computing the acceleration from the speed
    ext_horizon_steps = max(1, ceil(ext_horizon_time / dt));

    for i = 1:num_test_cases
        start_idx = (i - 1) * length_test_case + 1;
        end_idx = i * length_test_case + 1;
        
        %% TODO: size-1 cause hitch angle is currently not in data
        y = zeros(k.get_size() - 1, length_test_case + 1);
        %% TODO check dims and possibly transpose
        y(k.X, :) = rec_data.output.x(start_idx:end_idx);
        y(k.Y, :) = rec_data.output.y(start_idx:end_idx);
        y(k.V, :) = rec_data.output.v(start_idx:end_idx);
        y(k.PSI, :) = rec_data.output.psi(start_idx:end_idx);

        ref_times = 0:dt:length_test_case * dt;

        ref_states = zeros(k.get_size(), length_test_case + 1);
        ref_states(k.X, :) = rec_data.ref.x(start_idx:end_idx);
        ref_states(k.Y, :) = rec_data.ref.y(start_idx:end_idx);
        ref_states(k.V, :) = rec_data.ref.v(start_idx:end_idx);
        ref_states(k.PSI, :) = rec_data.ref.psi(start_idx:end_idx);

        ext_horizon_states = zeros(k.get_size(), ext_horizon_steps);
        ext_horizon_states(k.X, :) = rec_data.ref.x(end_idx:end_idx + ext_horizon_steps - 1);
        ext_horizon_states(k.Y, :) = rec_data.ref.y(end_idx:end_idx + ext_horizon_steps - 1);
        ext_horizon_states(k.V, :) = rec_data.ref.v(end_idx:end_idx + ext_horizon_steps - 1);
        ext_horizon_states(k.PSI, :) = rec_data.ref.psi(end_idx:end_idx + ext_horizon_steps - 1);
        ext_horizon_states(k.HA, :) = zeros(1, ext_horizon_steps);

        speeds = [ref_states(k.V, :), ext_horizon_states(k.V, :)];
        ref_inputs = zeros(inp_order.SIZE, length_test_case + 1);
        ref_inputs(inp_order.A, :) = arrayfun(@(i) (speeds(i + 1) - speeds(i)) / dt, 1:(length_test_case + 1));

        ref_traj = PredefinedTraj(ref_times, ref_inputs, ref_states, false, ext_horizon_states);

        [u, ~] = dyn_conf.create_params_for_random_sim(ref_traj, 1:(length_test_case + 1), dyn_conf_values);

        initial_state = y(:, 1);
        initial_state(k.HA) = 0;
        res{i} = testCase(y', u', initial_state', dt, dyn_class);
    end
end