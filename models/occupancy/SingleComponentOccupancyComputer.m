classdef SingleComponentOccupancyComputer < handle
    properties (Access = private)
        shape_set % zonotope <-- does not work
        shape_set_vertices
        f
        state_dims (:, 1) uint32
        set_transformer SetTransformer
    end

    methods

        function obj = SingleComponentOccupancyComputer(shape_set, create_f, ...
                state_order, occ_state_order, pos_order)
            obj.shape_set = shape_set;
            obj.shape_set_vertices = vertices(obj.shape_set);
            obj.f = create_f(occ_state_order, pos_order);
            obj.state_dims = occ_state_order.create_state_order_mapping(state_order);

            syms symb_state [occ_state_order.get_size() + pos_order.get_size() 1];
            obj.set_transformer = SetTransformer(obj.f(symb_state), symb_state);
        end

        function res = compute_occ(obj, r)
            r = project(r, obj.state_dims);
            if isa(r, 'zonotope') || isa(r, 'conZonotope')
                r = reduce(r, 'pca', 10);  % Aggressive reduction for low-memory demos
            end
            X = cartProd(r, obj.shape_set);
            % Reduce and over-approximate to keep generator count low
            if isa(X, 'zonotope') || isa(X, 'conZonotope')
                X = reduce(X, 'pca', 10);
                X = zonotope(interval(X));
            end
            res = obj.set_transformer.transform(X);
            if isa(res, 'zonotope') || isa(res, 'conZonotope')
                res = reduce(res, 'pca', 10);
                res = zonotope(interval(res));
            end
        end

        function res = compute_occ_for_single_state(obj, x)
            x = x(obj.state_dims);
            verts = obj.shape_set_vertices;
            res = arrayfun(@(i) obj.f([x; verts(:, i)]), 1:size(verts, 2), 'UniformOutput', false);
            res = cell2mat(res);
        end

    end
end
