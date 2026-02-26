classdef MaximumDirectionLengthComputer < handle & matlab.mixin.Heterogeneous
    properties (Access = private)
        occ_comp OccupancyComputer
        state_order AbstractStateOrder
        relevant_dims (1, :) uint32
        max_values_per_dim (1, :) double
        max_size_per_occ (:, 2) double
    end

    properties (Constant)
        punish_too_large = 100
        punish_too_small = 2
    end

    methods (Access = public)

        function obj = MaximumDirectionLengthComputer(occ_comp, state_order, ...
                relevant_dims, max_values_per_dim, max_size_per_occ)
            obj.occ_comp = occ_comp;
            obj.state_order = state_order;
            obj.relevant_dims = relevant_dims;
            obj.max_values_per_dim = max_values_per_dim;
            obj.max_size_per_occ = max_size_per_occ;
        end

        function fun = create_obj_fun(obj, c, direction)

            function cost = check_occ(scale)
                scaled_direction = scale * direction;

                full_c = zeros(obj.state_order.get_size(), 1);
                full_c(obj.relevant_dims) = c;
                full_dir = zeros(obj.state_order.get_size(), 1);
                full_dir(obj.relevant_dims) = scaled_direction;
                cz = conZonotope(full_c, [full_dir, -full_dir], [1 1], 1);

                occ = obj.occ_comp.compute_occupancy(cz);

                cost = 0;
                for i = 1:obj.occ_comp.num_shapes
                    o = occ{i};
                    [~, o_width,  ~, o_length] = calc_vectors_to_boundary(o);
                    values = [o_width, o_length];
                    for j = 1:length(values)
                        cost = cost + obj.check_occ_size(values(j), obj.max_size_per_occ(i, j));
                    end
                end
            end

            fun = @check_occ;
        end

    end

    methods (Access = private)

        function cost = check_occ_size(obj, occ_size, max_size)
            if occ_size > max_size
                cost = obj.punish_too_large * (occ_size - max_size);
            else
                cost = obj.punish_too_small * (max_size - occ_size);
            end
        end

    end
end
