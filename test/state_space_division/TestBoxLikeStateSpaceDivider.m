classdef TestBoxLikeStateSpaceDivider < matlab.unittest.TestCase
    methods (Test)

        function test_box_like_state_space_divider(testCase)
            k = KSTTrailerInpDeltaStateOrder();
            i = InpOrderADelta();

            result_dir = ResultDir('../../out/test_box_like_state_space_divider');

            %% Configs
            params = create_kst_trailer_params_MAN();
            state_meas_dist = meas_dist_for_kst_trailer_inp_delta_1(k);
            state_meas_dist_t = state_meas_dist';

            constraints = KSTTrailerInpDeltaConstraints(k, i, params);
            terminal_cond = StandstillTerminalCondition(k, 1);

            divider = create_base_initial_sampler_for_hierarchical_for_inp_delta( ...
                result_dir, 1, k, params, constraints, terminal_cond, state_meas_dist);

            [batch_idx, initial_sets] = divider.next_batch();

            testCase.verifyEqual(batch_idx, 1);
            testCase.verifySize(initial_sets, [1074 1]);

            state_1 = zeros(k.get_size(), 1);
            state_1(k.X) = 0;
            state_1(k.Y) = 0;
            state_1(k.V) = 20.0;
            state_1(k.HA) = 0.0;
            set_1 = interval(state_1 - state_meas_dist_t, state_1 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_1));

            state_2 = zeros(k.get_size(), 1);
            state_2(k.X) = 0;
            state_2(k.Y) = 0;
            state_2(k.V) = 20.0;
            state_2(k.HA) = 0.05;
            set_2 = interval(state_2 - state_meas_dist_t, state_2 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_2));

            state_3 = zeros(k.get_size(), 1);
            state_3(k.X) = 0;
            state_3(k.Y) = 0;
            state_3(k.V) = 20.0;
            state_3(k.HA) = -0.05;
            set_3 = interval(state_3 - state_meas_dist_t, state_3 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_3));

            state_4 = zeros(k.get_size(), 1);
            state_4(k.X) = 0;
            state_4(k.Y) = 0;
            state_4(k.V) = 15;
            state_4(k.HA) = 0.0;
            set_4 = interval(state_4 - state_meas_dist_t, state_4 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_4));

            state_5 = zeros(k.get_size(), 1);
            state_5(k.X) = 0;
            state_5(k.Y) = 0;
            state_5(k.V) = 15;
            state_5(k.HA) = 0.1;
            set_5 = interval(state_5 - state_meas_dist_t, state_5 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_5));

            state_6 = zeros(k.get_size(), 1);
            state_6(k.X) = 0;
            state_6(k.Y) = 0;
            state_6(k.V) = 15;
            state_6(k.HA) = -0.1;
            set_6 = interval(state_6 - state_meas_dist_t, state_6 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_6));

            state_7 = zeros(k.get_size(), 1);
            state_7(k.X) = 0;
            state_7(k.Y) = 0;
            state_7(k.V) = 10;
            state_7(k.HA) = -0.1;
            set_7 = interval(state_7 - state_meas_dist_t, state_7 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_7));

            state_8 = zeros(k.get_size(), 1);
            state_8(k.X) = 0;
            state_8(k.Y) = 0;
            state_8(k.V) = 4;
            state_8(k.HA) = 0.15;
            set_8 = interval(state_8 - state_meas_dist_t, state_8 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_8));

            state_9 = zeros(k.get_size(), 1);
            state_9(k.X) = 0;
            state_9(k.Y) = 0;
            state_9(k.V) = 1;
            state_9(k.HA) = -0.2;
            set_9 = interval(state_9 - state_meas_dist_t, state_9 + state_meas_dist_t);
            testCase.verifyTrue(find_set_containing_state(initial_sets, set_9));
        end

    end
end

function set_found = find_set_containing_state(initial_sets, state)
    set_found = false;
    for i = 1:length(initial_sets)
        s = initial_sets{i};
        if s.cora_set.contains(state)
            set_found = true;
            return
        end
    end
end
