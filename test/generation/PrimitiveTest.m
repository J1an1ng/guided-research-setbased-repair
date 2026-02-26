classdef PrimitiveTest < matlab.unittest.TestCase
    properties
        p1_start_set_1_traj_1 Primitive
        p2_start_set_1_traj_1 Primitive
        p_start_set_1_traj_2 Primitive
        p_start_set_2_traj_1 Primitive
    end

    methods (TestMethodSetup)

        function create_primitives(testCase)
            k = KSTTrailerStateOrderDefault;
            i = InpOrderADeltaVDefault;
            create_inp_traj = @(a, dv) KSTConstantInpTrajADeltaV(k, i, a, dv);
            create_inp_traj_1 = @() create_inp_traj(-3.5, 0.1);
            create_inp_traj_2 = @() create_inp_traj(-4.0, 0.1);
            create_itv_1 = @() interval([-2; -5; -10; 0], [2; 5; 10; 3]);
            create_itv_2 = @() interval([-2; -5; -10; 1], [2; 5; 10; 3]);
            create_start_set_1 = @() SetContainer(1, create_itv_1(), true);
            create_start_set_2 = @() SetContainer(2, create_itv_1(), false, create_itv_2());
            dt = 0.1;
            occ_dt = 0.1;

            testCase.p1_start_set_1_traj_1 = Primitive(create_start_set_1(), create_inp_traj_1(), dt, occ_dt);
            testCase.p2_start_set_1_traj_1 = Primitive(create_start_set_1(), create_inp_traj_1(), dt, occ_dt);
            testCase.p_start_set_1_traj_2 = Primitive(create_start_set_1(), create_inp_traj_2(), dt, occ_dt);
            testCase.p_start_set_2_traj_1 = Primitive(create_start_set_2(), create_inp_traj_1(), dt, occ_dt);
        end

    end

    methods (Test)

        function test_jsonencode(testCase)
            act = jsonencode(testCase.p1_start_set_1_traj_1);
            exp = '{"start_set_id":1,"ref_traj":{"a":-3.5,"delta_v":0.1}}';
            testCase.verifyEqual(act, exp);
        end

        function test_hash(testCase)
            testCase.verifyEqual(keyHash(testCase.p1_start_set_1_traj_1), ...
                keyHash(testCase.p2_start_set_1_traj_1));
            testCase.verifyNotEqual(keyHash(testCase.p1_start_set_1_traj_1), ...
                keyHash(testCase.p_start_set_1_traj_2));
            testCase.verifyNotEqual(keyHash(testCase.p1_start_set_1_traj_1), ...
                keyHash(testCase.p_start_set_2_traj_1));
        end

    end
end
