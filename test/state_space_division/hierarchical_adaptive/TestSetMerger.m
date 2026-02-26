classdef TestSetMerger < matlab.unittest.TestCase
    methods (Test)

        function test_merge_2d(testCase)
            id_man = SetContainerIdManager();
            sets = create_2d_sets(3, 0.6, id_man);

            el = ellipsoid([0.2 0; 0 0.2], [4; 2]);
            [sets, s_id_loose] = add_set(sets, el, id_man);

            get_scales = @(c) [2; 1];
            radius_scale_ext = 0.1;
            check_size_of_sets = create_check_size_of_sets([0.7; 0.6]);
            merger = SetMerger([1 2], get_scales, radius_scale_ext, check_size_of_sets);

            new_sets = dictionary();

            % should be merged with set 1
            el = ellipsoid([0.1 0; 0 0.1], [1.1; 0.9]);
            [new_sets, s_id_1] = add_set(new_sets, el, id_man);

            % should be merged together and with set 2
            el = ellipsoid([0.1 0; 0 0.1], [2.2; 1.1]);
            [new_sets, s_id_2] = add_set(new_sets, el, id_man);
            el = ellipsoid([0.1 0; 0 0.1], [2.2; 0.9]);
            [new_sets, s_id_3] = add_set(new_sets, el, id_man);

            % should be merged with loose set
            el = ellipsoid([0.1 0; 0 0.1], [4; 2.2]);
            [new_sets, s_id_4] = add_set(new_sets, el, id_man);

            % should not be merged
            el = ellipsoid([0.1 0; 0 0.1], [1.5; 2.5]);
            [new_sets, s_id_5] = add_set(new_sets, el, id_man);

            % should be merged with set 3 and enlargen it
            el = ellipsoid([0.45 0; 0 0.45], [3; 1]);
            [new_sets, s_id_6] = add_set(new_sets, el, id_man);

            % figure;
            % hold on;
            % plot_all(sets);
            % plot_all(new_sets);

            [new_current_sets, subset_relation] = merger.merge(new_sets, sets, id_man);

            % figure;
            % hold on;
            % plot_all(new_current_sets);

            set_id = 1;
            new_id = subset_relation(set_id);
            new_set = new_current_sets(new_id);
            testCase.verifyEqual(subset_relation(s_id_1), new_id);
            testCase.verifyEqual(new_set.cora_set, sets(set_id).cora_set);
            testCase.verifyTrue(new_set.has_inner_base_set);
            testCase.verifyEqual(new_set.inner_base_set, sets(set_id).cora_set);
            testCase.verifyTrue(contains(new_set.cora_set, new_sets(s_id_1).cora_set));

            set_id = 2;
            new_id = subset_relation(set_id);
            new_set = new_current_sets(new_id);
            testCase.verifyEqual(subset_relation(s_id_2), new_id);
            testCase.verifyEqual(subset_relation(s_id_3), new_id);
            testCase.verifyEqual(new_set.cora_set, sets(set_id).cora_set);
            testCase.verifyTrue(new_set.has_inner_base_set);
            testCase.verifyEqual(new_set.inner_base_set, sets(set_id).cora_set);
            testCase.verifyTrue(contains(new_set.cora_set, new_sets(s_id_2).cora_set));
            testCase.verifyTrue(contains(new_set.cora_set, new_sets(s_id_3).cora_set));

            new_id = subset_relation(s_id_loose);
            new_set = new_current_sets(new_id);
            testCase.verifyEqual(subset_relation(s_id_4), new_id);
            testCase.verifyFalse(new_set.has_inner_base_set);
            testCase.verifyEqual(new_set.inner_base_set, []);
            testCase.verifyTrue(contains(new_set.cora_set, sets(s_id_loose).cora_set));
            testCase.verifyTrue(contains(new_set.cora_set, new_sets(s_id_4).cora_set));

            testCase.verifyFalse(isKey(subset_relation, s_id_5));

            set_id = 3;
            new_id = subset_relation(set_id);
            new_set = new_current_sets(new_id);
            testCase.verifyEqual(subset_relation(s_id_6), new_id);
            testCase.verifyTrue(new_set.has_inner_base_set);
            testCase.verifyEqual(new_set.inner_base_set, sets(set_id).cora_set);
            testCase.verifyTrue(contains(new_set.cora_set, sets(set_id).cora_set, 'approx'));
            testCase.verifyTrue(contains(new_set.cora_set, new_sets(s_id_6).cora_set, 'approx'));
        end

    end
end

% function plot_all(sets)
%    sets_values = values(sets);
%    for i = 1:length(sets_values)
%        s = sets_values(i);
%        plot(s.cora_set, [1 2]);
%    end
% end

function fun = create_check_size_of_sets(max_rads)

    function [res, set_union] = check_size_of_sets(sets)
        itv_union = fold(@(a, b) a.enclosing_itv | b.enclosing_itv, sets);
        set_union = fold(@(a, b) a.cora_set | b.cora_set, sets);
        r = rad(itv_union);
        EPS = 0.0001;
        res = all(r - EPS <= max_rads);
    end

    fun = @check_size_of_sets;
end

function [sets, s_id] = add_set(sets, el, id_man)
    z = zonotope(el, 3, 'inner:norm');
    s_id = id_man.gen_next_id();
    s = SetContainer(s_id, z, false);
    sets(s_id) = s;
end
