classdef (Abstract) AbstractOnceStateSpaceDivider < AbstractStateSpaceDivider
    methods (Abstract)
        all_batch = all_batches(obj)
    end

    properties (SetAccess = immutable, GetAccess = private)
        num_workers uint32
        search_dims (:, 1) uint32
    end

    properties (SetAccess = private, GetAccess = private)
        k_d_tree = []
    end

    methods

        function obj = AbstractOnceStateSpaceDivider(result_dir, num_workers, ...
                search_dims)
            obj = obj@AbstractStateSpaceDivider(result_dir);
            obj.num_workers = num_workers;
            obj.search_dims = search_dims;
        end

        function after_primitives_generated(obj, primitives)
            % It allows using a custom distance function (we could use one weighting
            % the dimensions differently).

            obj.create_tree();

            final_set_centers = cellfun(@(p) p.final_set.c, primitives, 'UniformOutput', false);
            final_set_centers = [final_set_centers{:}];
            final_set_centers = final_set_centers(obj.search_dims, :)';
            %% FIXME maybe increase the 4...currently, some primitive end sets are mapped to 0,
            %% which means that no enclosing set was found
            nns = knnsearch(obj.k_d_tree, final_set_centers, 'K', 10);

            res = cell(length(primitives), 1);
            b = obj.current_batch;
            sub_batches = arrayfun(@(i) {b(nns(i, :))}, 1:length(primitives));
            if obj.num_workers > 1
                parfor i = 1:length(primitives)
                    p = primitives{i};
                    sub_batch = sub_batches{i};
                    res{i} = find_enclosing_set(sub_batch, p.final_set);
                end
            else
                for i = 1:length(primitives)
                    if mod(i, 10) == 0
                        fprintf('Processing primitive %d/%d\n', i, length(primitives));
                    end
                    p = primitives{i};
                    sub_batch = sub_batches{i};
                    res{i} = find_enclosing_set(sub_batch, p.final_set);
                end
            end

            for i = 1:length(primitives)
                p = primitives{i};
                % TODO throw error if there is already a key with the same id
                obj.subset_relation(p.final_set.id) = res{i};
            end
        end

    end

    methods (Access = protected)

        function batch = generate_next_batch(obj)
            if obj.current_batch_idx == 1
                batch = obj.all_batches();
            else
                batch = [];
            end
        end

        function create_tree(obj)
            if isempty(obj.k_d_tree)
                centers = cellfun(@(x) x.c, obj.current_batch, 'UniformOutput', false);
                centers = [centers{:}];
                centers = centers(obj.search_dims, :)';
                obj.k_d_tree = ExhaustiveSearcher(centers); % , 'Distance', 'seuclidean');
            end
        end

    end
end

function set_id = find_enclosing_set(batch, final_set)
    for k = 1:length(batch)
        s = batch{k};
        %% little hack to account for Matlab's weird equality behavior
        if s.cora_set.contains(0.999 * final_set.cora_set)
            set_id = s.id;
            return
        end
    end
    set_id = 0;
end
