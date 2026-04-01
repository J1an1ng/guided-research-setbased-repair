function test_suite = generate_synthetic_test_suite( ...
    initial_set_sampler, valid_inp_traj_generator, dyn, dyn_conf, dyn_conf_values, num_sims, dt, length_test_case, initial_itv_sizes)
    %% TODO for different intial states generate some ref traj for each
    [~, initial_sets] = initial_set_sampler.next_batch();

    test_suite = {};
    for i = 1:length(initial_sets)
        initial_set = initial_sets{i}.cora_set;
        initial_state = center(initial_set);
        [trajectories, ~] = valid_inp_traj_generator.generate(initial_state);
        for j = 1:length(trajectories)
            ref_traj = trajectories{j};
            steps = size(ref_traj.inputs, 2);
            params = ref_traj.create_cora_sim_params(dyn_conf, dyn_conf_values, initial_itv_sizes);
            sim_res = simulateRandom(dyn, params, create_random_sim_options(num_sims, steps));

            num_test_cases = floor(size(ref_traj.inputs, 2) / length_test_case);

            % we do not want to use the last state as due to a bug in Cora, the last output is computed not using
            % the last input but the second last one
            assert(num_test_cases * length_test_case < size(ref_traj.inputs, 2));

            new_test_cases = cell(num_test_cases, 1);

            for k = 1:num_test_cases
                % We extract a time window from each simulation result
                start_idx = (k - 1) * length_test_case + 1;
                % We do not use that state as due to a bug in Cora, the last output is computed not using
                % the last input but the second last one
                end_idx = k * length_test_case + 1;

                % Get dimensions from first simulation result
                first_y = sim_res(1).y{1};
                num_states = size(first_y, 2);

                % Pre-allocate array for all trajectories
                y = zeros(length_test_case + 1, num_states, length(sim_res));

                % Fill array with trajectory segments from each simulation
                for sim_idx = 1:length(sim_res)
                    %% Hint: the first element in sim_res is the state after the first input (i.e., not the initial state);
                    %% to make things easier, we do not use the initial state
                    y(:, :, sim_idx) = sim_res(sim_idx).y{1}(start_idx:end_idx, :);
                end

                new_test_cases{k} = testCase(y, params.u(:, start_idx:end_idx)', initial_state', dt, class(dyn));
            end
            test_suite = [test_suite; new_test_cases];
        end
    end
end

function options = create_random_sim_options(num_sims, nr_const_inp)
    options.points = num_sims;
    options.fracVert = 0.5;
    options.fracInpVert = 0.5;
    options.nrConstInp = nr_const_inp;
end
