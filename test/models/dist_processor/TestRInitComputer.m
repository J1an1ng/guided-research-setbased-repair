classdef TestRInitComputer < matlab.unittest.TestCase
    methods (Test)

        function compute_initial_set(testCase)
            k = KSTTrailerStateOrder();
            comp = KSTRInitComputerForRotatedPositionalDists(k);
            state_meas_dist = meas_dist_for_kst_trailer_final(k);
            state_meas_dist(k.PSI) = pi / 8;
            state_meas_dist_itv = interval(-state_meas_dist, state_meas_dist);
            r_init = comp.compute_initial_set([0; 0; 0; 20; 0; 0], state_meas_dist_itv);

            % plot(r_init, 1:2);
            r_init_itv = interval(r_init);
            rads = rad(r_init_itv);

            testCase.assumeGreaterThanOrEqual(rads(k.X), state_meas_dist(k.X));
            testCase.assumeGreaterThanOrEqual(rads(k.Y), state_meas_dist(k.Y));
            testCase.verifyNotEqual(rads(k.X), state_meas_dist(k.X));
            testCase.verifyNotEqual(rads(k.Y), state_meas_dist(k.Y));

            testCase.verifyEqual(rads(k.DELTA), state_meas_dist(k.DELTA), AbsTol = 1e3);
            testCase.verifyEqual(rads(k.V), state_meas_dist(k.V), AbsTol = 1e3);
            testCase.verifyEqual(rads(k.PSI), state_meas_dist(k.PSI), AbsTol = 1e3);
            testCase.verifyEqual(rads(k.HA), state_meas_dist(k.HA), AbsTol = 1e3);
        end

    end
end
