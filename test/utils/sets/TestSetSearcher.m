classdef TestSetSearcher < matlab.unittest.TestCase
    methods (Test)

        function test_searcher_2d_simple(testCase)
            sets = create_2d_sets(10, 1);

            get_scales = @(c) [1; 1];
            radius_scale_ext = 0.1;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(1)), uint32([1 2 11 12]));
            testCase.verifyEqual(searcher.search(sets(2)), uint32([2 1 3 12 11 13]));
            testCase.verifyEqual(searcher.search(sets(10)), uint32([10 9 20 19]));
            testCase.verifyEqual(searcher.search(sets(45)), uint32([45 35 44 46 55 34 36 54 56]));
        end

        function test_searcher_2d_with_one_scale(testCase)
            sets = create_2d_sets(10, 1);

            get_scales = @(c) [2; 2];
            radius_scale_ext = 0.1;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(1)), uint32([1 2 11 12 3 21 13 22 23]));
            testCase.verifyEqual(searcher.search(sets(2)), uint32([2 1 3 12 11 13 4 22 14 21 23 24]));
            testCase.verifyEqual(searcher.search(sets(10)), uint32([10 9 20 19 8 30 18 29 28]));
            testCase.verifyEqual(searcher.search(sets(45)), uint32([45 35 44 46 55 34 36 54 56 25 ...
                                                                    43 47 65 24 26 33 37 53 57 64 ...
                                                                    66 23 27 63 67]));
        end

        function test_searcher_2d_with_diff_scale(testCase)
            sets = create_2d_sets(10, 1);

            get_scales = @(c) [2.01; 1];
            radius_scale_ext = 0.01;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(1)), uint32([1 2 3 11 12 13]));
            testCase.verifyEqual(searcher.search(sets(2)), uint32([2 1 3 4 12 11 13 14]));
            testCase.verifyEqual(searcher.search(sets(10)), uint32([10 9 8 20 19 18]));
            testCase.verifyEqual(searcher.search(sets(45)), uint32([45 44 46 43 47 35 55 34 36 54 ...
                                                                    56 33 37 53 57]));
        end

        function test_searcher_2d_with_rad_ext(testCase)
            sets = create_2d_sets(10, 1);

            get_scales = @(c) [1; 1];
            radius_scale_ext = 1;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(1)), uint32([1 2 11 12 3 21]));
            testCase.verifyEqual(searcher.search(sets(2)), uint32([2 1 3 12 11 13 4 22]));
            testCase.verifyEqual(searcher.search(sets(10)), uint32([10 9 20 19 8 30]));
            testCase.verifyEqual(searcher.search(sets(45)), ...
                uint32([45 35 44 46 55 34 36 54 56 25 43 47 65]));
        end

        function test_searcher_2d_with_larger_search_set(testCase)
            id_man = SetContainerIdManager();
            sets = create_2d_sets(10, 1, id_man);

            get_scales = @(c) [1; 1];
            radius_scale_ext = 0.01;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(scale_set(sets(1), 2, id_man.gen_next_id())), ...
                uint32([1 2 11 12 3 21 13 22 23]));
            testCase.verifyEqual(searcher.search(scale_set(sets(2), 2, id_man.gen_next_id())), ...
                uint32([2 1 3 12 11 13 4 22 14 21 23 24]));
            testCase.verifyEqual(searcher.search(scale_set(sets(10), 2, id_man.gen_next_id())), ...
                uint32([10 9 20 19 8 30 18 29 28]));
            testCase.verifyEqual(searcher.search(scale_set(sets(45), 2, id_man.gen_next_id())), ...
                uint32([45 35 44 46 55 34 36 54 56 25 43 47 65 24 26 33 37 53 57 64 66 23 27 63 67]));
        end

        function test_searcher_3d(testCase)
            sets = create_3d_sets(10, 1);

            get_scales = @(c) [1; 1; 1];
            radius_scale_ext = 0.1;
            searcher = SetSearcher(sets, [1 2 3], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(1)), ...
                uint32([1 2 11 101 12 102 111 112]));
            testCase.verifyEqual(searcher.search(sets(2)), ...
                uint32([2 1 3 12 102 11 13 101 103 112 111 113]));
            testCase.verifyEqual(searcher.search(sets(10)), ...
                uint32([10 9 20 110 19 109 120 119]));
            testCase.verifyEqual(searcher.search(sets(45)), ...
                uint32([45 35 44 46 55 145 34 36 54 56 135 144 146 155 134 136 154 156]));
            testCase.verifyEqual(searcher.search(sets(245)), ...
                uint32([245 145 235 244 246 255 345 135 144 146 155 234 236 ...
                        254 256 335 344 346 355 134 136 154 156 334 336 354 356]));
        end

        function test_searcher_3d_sub_dims(testCase)
            sets = create_3d_sets(10, 1);

            get_scales = @(c) [1; 1];
            radius_scale_ext = 0.1;
            searcher = SetSearcher(sets, [1 2], get_scales, radius_scale_ext);

            testCase.verifyEqual(searcher.search(sets(245)), ...
                uint32([45, 145, 245, 345, 445, 545, 645, 745, 845, 945, ...
                        35, 44, 46, 55, 135, 144, 146, 155, 235, 244, 246, 255, ...
                        335, 344, 346, 355, 435, 444, 446, 455, 535, 544, 546, 555, ...
                        635, 644, 646, 655, 735, 744, 746, 755, 835, 844, 846, 855, ...
                        935, 944, 946, 955, 34, 36, 54, 56, 134, 136, 154, 156, 234, ...
                        236, 254, 256, 334, 336, 354, 356, 434, 436, 454, 456, 534, ...
                        536, 554, 556, 634, 636, 654, 656, 734, 736, 754, 756, 834, ...
                        836, 854, 856, 934, 936, 954, 956]));
        end

    end
end
