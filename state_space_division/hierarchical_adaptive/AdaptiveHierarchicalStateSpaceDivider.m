classdef AdaptiveHierarchicalStateSpaceDivider < AbstractStateSpaceDivider
    properties (Constant)
        property_names_to_dump = {'current_sets'; 'too_large_current_sets'}
    end

    properties (GetAccess = private, SetAccess = immutable)
        search_dims (1, :) uint32

        get_scales
        maximum_relative_growth double

        hierarchical_dim uint32
        hierarchical_dim_min_step_size double
        hierarchical_dim_offset double

        base_divider AbstractOnceStateSpaceDivider
        set_merger SetMerger
    end

    % Hint: these properties should actually be private, but we need to
    % read and write them from the base class. Somehow, Matlab does not
    % allow to access a private/protected property from a base class.
    properties (Access = public)
        current_sets dictionary = dictionary()
        % TODO update after each batch generation, i.e., remove the used sets
        too_large_current_sets dictionary = dictionary()
        % TODO update after each batch generation, i.e., remove the used sets
    end

    methods

        function obj = AdaptiveHierarchicalStateSpaceDivider(result_dir, ...
                hierarchical_dim, hierarchical_dim_min_step_size, hierarchical_dim_offset, ...
                base_divider, set_merger)
            obj = obj@AbstractStateSpaceDivider(result_dir);
            obj.hierarchical_dim = hierarchical_dim;
            obj.hierarchical_dim_min_step_size = hierarchical_dim_min_step_size;
            obj.hierarchical_dim_offset = hierarchical_dim_offset;
            obj.base_divider = base_divider;
            obj.set_merger = set_merger;

            obj.load_dump();
            obj.base_divider.set_set_id_manager(obj.set_id_manager);
        end

        function after_primitives_generated(obj, primitives)
            primitives = primitives(cellfun(@(p) ~p.is_terminal, primitives));
            final_sets = cellfun(@(p) p.final_set, primitives, 'UniformOutput', false);

            final_sets_dict = to_dict(final_sets);

            fprintf('#New final sets: %d. \n', final_sets_dict.numEntries);
            fprintf('#Current sets: %d. \n', obj.current_sets.numEntries);
            fprintf('#Total: %d. \n', obj.current_sets.numEntries + final_sets_dict.numEntries);

            % TODO do not parametrize the merge_sets function, just use
            % merge_sets_iteratively always...or parametrize it but then
            % as an object with its own config already inside
            [new_current_sets, add_subset_relation] = obj.set_merger.merge( ...
                final_sets_dict, obj.current_sets, obj.set_id_manager);
            obj.current_sets = new_current_sets;

            fprintf('#Current sets after merge: %d. \n', obj.current_sets.numEntries);

            assert (~any(arrayfun(@(x) isKey(obj.subset_relation, x), add_subset_relation.keys())));
            new_subset_relation = insert(obj.subset_relation, ...
                add_subset_relation.keys(), add_subset_relation.values());
            obj.subset_relation = unnest_dict(new_subset_relation);
        end

        function get_max_final_set(~, ~)
            error('Not implemented');
        end

    end

    methods (Access = protected)

        function batch = generate_next_batch(obj)
            if obj.current_batch_idx == 1
                assert(obj.current_sets.numEntries == 0);
                assert(obj.too_large_current_sets.numEntries == 0);
                [~, base_sets] = obj.base_divider.next_batch();
                new_base_sets_dict = to_dict(base_sets);
                obj.current_sets = insert(obj.current_sets, ...
                    new_base_sets_dict.keys(), new_base_sets_dict.values());
                % The base sets should not already be too large:
                assert(obj.too_large_current_sets.numEntries == 0);
                % The id managers should be in sync:
                assert(max(cellfun(@(x) x.id, base_sets)) == obj.set_id_manager.get_current_id());
            end
            res_in_current_sets = obj.next_batch_in_current_sets();
            res_too_large_current_sets = obj.too_large_current_sets.values('cell');
            obj.too_large_current_sets = dictionary();
            batch = [res_in_current_sets; res_too_large_current_sets];
        end

    end

    methods (Access = private)

        function res = next_batch_in_current_sets(obj)
            current_sets_entries = obj.current_sets.entries();
            current_sets_entries.c_hierrachical_dim = arrayfun( ...
                @(x) x.c(obj.hierarchical_dim), current_sets_entries.Value);
            [~, max_idx] = max(current_sets_entries.c_hierrachical_dim);
            max_hierarchical_value = current_sets_entries.c_hierrachical_dim(max_idx);
            threshold = max_hierarchical_value + ...
                obj.hierarchical_dim_min_step_size + obj.hierarchical_dim_offset;

            fprintf('#Velocity threshold for next batch: %d. \n', threshold);

            idcs = current_sets_entries.Key(current_sets_entries.c_hierrachical_dim > threshold);
            res = num2cell(obj.current_sets(idcs));
            obj.current_sets = obj.current_sets.remove(idcs);
        end

    end
end

function res = ids(set_containers)
    res = arrayfun(@(x) x.id, set_containers);
end

function new_sets_dict = to_dict(new_sets)
    new_sets = vertcat(new_sets{:});
    new_sets_dict = dictionary(ids(new_sets), new_sets);
end
