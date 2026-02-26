classdef SetMerger < handle & matlab.mixin.Heterogeneous
    properties (SetAccess = immutable, GetAccess = private)
        search_dims (1, :) uint32
        get_scales
        maximum_relative_growth double
        check_size_of_sets
    end

    methods

        function obj = SetMerger(search_dims, get_scales, maximum_relative_growth, check_size_of_sets)
            obj.search_dims = search_dims;
            obj.get_scales = get_scales;
            obj.maximum_relative_growth = maximum_relative_growth;
            obj.check_size_of_sets = check_size_of_sets;
        end

        function [new_current_sets, subset_relation] = merge(obj, new_sets, current_sets, set_id_manager)
            current_keys = current_sets.keys();
            current_values = current_sets.values();
            has_inner_base = arrayfun(@(s) s.has_inner_base_set, current_values);
            current_base_sets_keys = current_keys(has_inner_base);
            current_loose_sets_keys = current_keys(~has_inner_base);
            current_base_sets = dictionary(current_base_sets_keys, current_values(has_inner_base));
            current_loose_sets = dictionary(current_loose_sets_keys, current_values(~has_inner_base));

            new_sets_done = dictionary(uint32.empty, SetContainer.empty);

            subset_relation = dictionary(uint32.empty, uint32.empty);

            searcher_new = SetSearcher(new_sets, obj.search_dims, obj.get_scales, ...
                obj.maximum_relative_growth);
            searcher_loose = SetSearcher(current_loose_sets, obj.search_dims, obj.get_scales, ...
                obj.maximum_relative_growth);
            searcher_base = SetSearcher(current_base_sets, obj.search_dims, obj.get_scales, ...
                obj.maximum_relative_growth);
            counter = 0;
            num_sets_to_merge = new_sets.numEntries;

            while new_sets.numEntries > 0
                new_sets_ids = new_sets.keys();
                i = new_sets_ids(1);
                s = new_sets(i);
                new_sets(i) = [];
                max_rads_of_merged_sets = rad(s.enclosing_itv);

                ks = searcher_new.search(s);
                for k = ks
                    if isKey(new_sets, k)
                        [~, i, s, new_sets, subset_relation, ...
                            max_rads_of_merged_sets] = obj.try_to_merge( ...
                            i, s, k, new_sets, set_id_manager, subset_relation, max_rads_of_merged_sets);
                    end
                end

                success = false;

                ks = searcher_loose.search(s);
                for k = ks
                    if isKey(current_loose_sets, k)
                        [success, i, s, current_loose_sets, subset_relation, ...
                            max_rads_of_merged_sets] = obj.try_to_merge(i, s, k, ...
                            current_loose_sets, set_id_manager, subset_relation, max_rads_of_merged_sets);
                    end
                    if success
                        % if a merge was successfull, we know that no further merge
                        % is possible, as all other loose sets were tried with each others
                        % already...
                        break
                    end
                end

                if ~success
                    % ...same here.
                    ks = searcher_base.search(s);
                    for k = ks
                        if isKey(current_base_sets, k)
                            [success, i, s, current_base_sets, subset_relation, ...
                                max_rads_of_merged_sets] = obj.try_to_merge( ...
                                i, s, k, current_base_sets, set_id_manager, ...
                                subset_relation, max_rads_of_merged_sets);
                        end
                        if success
                            % We do not want to merge two sets with inner base sets, as the overall base-set
                            % might not be an interval anymore.
                            break
                        end
                    end
                end

                new_sets_done(i) = s;
                counter = counter + 1;
                if mod(counter, 100) == 0
                    fprintf('Merged sets: %d/%d. Left: %d\n', counter, num_sets_to_merge, ...
                        new_sets.numEntries);
                end
            end
            new_current_sets = dictionary( ...
                [current_base_sets.keys(); current_loose_sets.keys(); new_sets_done.keys()], ...
                [current_base_sets.values(); current_loose_sets.values(); new_sets_done.values()]);
            subset_relation = unnest_dict(subset_relation);
        end

    end

    methods (Access = private)

        function [success, i, s, sets, subset_relation, ...
                max_rads_of_merged_sets] = try_to_merge(obj, i, s, k, ...
                sets, set_id_manager, subset_relation, max_rads_of_merged_sets)
            t = sets(k);
            assert(~(s.has_inner_base_set && t.has_inner_base_set), ...
                ['We do not want to merge if both sets have inner base sets', ...
                 '(as the overall base-set might not be an interval anymore)']);
            success = false;
            % we check this as the union of a zonotope and an enclosing
            % interval might not be an interval anymore
            if isa(s.cora_set, 'interval') && contains(s.cora_set, t.cora_set)
                s_t_union = s.cora_set;
                do_merge = true;
            elseif isa(t.cora_set, 'interval') && contains(t.cora_set, s.cora_set)
                s_t_union = t.cora_set;
                do_merge = true;
            else
                max_rads_sets = max([rad(t.enclosing_itv) max_rads_of_merged_sets], [], 2);
                [do_merge, s_t_union] = obj.check_size_of_sets({s, t}, max_rads_sets);
            end

            if do_merge
                inner_base_set = [];
                if s.has_inner_base_set
                    inner_base_set = s.inner_base_set;
                elseif t.has_inner_base_set
                    inner_base_set = t.inner_base_set;
                end
                merged_set = SetContainer(set_id_manager.gen_next_id(), ...
                    s_t_union, false, inner_base_set);
                sets(k) = [];
                subset_relation(i) = merged_set.id;
                subset_relation(k) = merged_set.id;
                i = merged_set.id;
                s = merged_set;
                max_rads_of_merged_sets = max_rads_sets;
                success = true;
            end
        end

    end
end
