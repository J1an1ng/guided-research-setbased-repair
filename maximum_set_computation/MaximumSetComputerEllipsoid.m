classdef MaximumSetComputerEllipsoid < handle
    properties (Access = private)
        state_order AbstractStateOrder
        % dimensions that have influence on the occupancy
        relevant_dims (1, :) uint32
        desired_rads_per_dim (1, :) double
        max_size_per_occ (:, 2) double
        vec_size uint32
        fun_q_from_vec
        out_path string
        occ_comp OccupancyComputer %% FIXME remove again
        iteration uint32
    end

    properties (Constant)
        punish_too_large = 1000 %% TODO probably has to be increased
        % 70 already ok but occ a bit too large
        punish_rad_too_small = 5

        % We slightly scale the optimized ellipsoid as the scaled
        % version still works
        scale = 1.1

        % To get the maximum set (not the maximum merge set but the
        % the set defining when a final set is definitely too large),
        % we scale the optimized ellipsoid a bit more
        scale_for_total = 1.5
    end

    methods

        function obj = MaximumSetComputerEllipsoid(k, relevant_dims, occ_comp, ...
                desired_rads_per_dim, max_size_per_occ, ...
                vec_size, fun_q_from_vec, ...
                out_path, iteration)
            if isempty(fun_q_from_vec)
                fun_q_from_vec = @q_from_vec;
            end

            obj.state_order = k;
            obj.relevant_dims = relevant_dims;
            obj.occ_comp = occ_comp;
            obj.desired_rads_per_dim = desired_rads_per_dim;
            obj.max_size_per_occ = max_size_per_occ;
            obj.vec_size = vec_size;
            obj.fun_q_from_vec = fun_q_from_vec;
            obj.out_path = out_path;
            obj.iteration = iteration;
        end

        function compute_max_set(obj, centers)
            Q_vec_out_file = obj.out_path + "/el_opt_res_" + obj.iteration + ".mat";

            if isfile(Q_vec_out_file)
                load(Q_vec_out_file, 'Q_vec');
            else
                fun = obj.create_objective_fun(centers);
                lb = zeros(obj.vec_size, 1);
                ub = 10 * ones(obj.vec_size, 1);
                out_file_checkpoint = obj.out_path + "/checkpt_" + obj.iteration + ".mat";
                opts = optimoptions('surrogateopt', ...
                    'CheckpointFile', out_file_checkpoint, ...
                    'UseParallel', true, 'PlotFcn', 'surrogateoptplot', ...
                    'MaxFunctionEvaluations', 1000, ...
                    'MinSurrogatePoints', 60);

                Q_vec = surrogateopt(fun, lb, ub, opts);

                save(Q_vec_out_file, 'Q_vec');
            end

            c = zeros(length(obj.relevant_dims), 1);
            [~, z, ~] = obj.create_sets(Q_vec, c);
            obj.plot_occ(z);

            z = obj.scale * z;
            obj.plot_occ(z);
            z_out_file = obj.out_path + "/max_z_" + obj.iteration + ".mat";
            save(z_out_file, 'z');

            max_itv = interval(z);
            max_itv_out_file = obj.out_path + "/max_itv_" + obj.iteration + ".mat";
            save(max_itv_out_file, 'max_itv');

            max_poly = polytope(z);
            poly_out_file = obj.out_path + "/max_poly_" + obj.iteration + ".mat";
            save(poly_out_file, 'max_poly');

            z_total = obj.scale_for_total * z;
            obj.plot_occ(z_total);
            max_poly = polytope(z_total);
            total_poly_out_file = obj.out_path + "/max_total_poly_" + obj.iteration + ".mat";
            save(total_poly_out_file, 'max_poly');

            max_total_itv = interval(z_total);
            max_total_itv_out_file = obj.out_path + "/max_total_itv_" + obj.iteration + ".mat";
            save(max_total_itv_out_file, 'max_total_itv');
        end

        function plot_occ(obj, z)
            zz = projectHighDim(z, obj.state_order.get_size(), obj.relevant_dims);
            figure;
            occ = obj.occ_comp.compute_occupancy(zz);
            truck_occ = occ{1};
            trailer_occ = occ{2};
            plot(truck_occ, [1 2]);
            hold on;
            plot(trailer_occ, [1 2]);
            axis equal;
        end

    end

    methods (Access = private)

        function z = extend_position_set(obj, z)
            %% makes the projection onto the position domain
            %% interval like
            n = length(obj.relevant_dims);
            pos_dims = [1 2];
            pos_itv = interval(project(z, pos_dims));
            rem_dims = n - length(pos_dims);
            pos_itv_ext = cartProd(pos_itv, ...
                interval(zeros(rem_dims, 1), zeros(rem_dims, 1)));
            z = z | pos_itv_ext;
        end

        function [el, z, zz] = create_sets(obj, Q_vec, c)
            Q = obj.fun_q_from_vec(length(obj.relevant_dims), Q_vec);
            el = ellipsoid(Q, c);
            z = zonotope(el, 20, 'inner:norm');
            zz = projectHighDim(z, obj.state_order.get_size(), obj.relevant_dims);
        end

        function fun = create_objective_fun(obj, centers)

            function cost = obj_fun(Q_vec)
                cost = 1e7;
                acc_occ_cost = 0;
                num_centers = size(centers, 2);
                for k = 1:num_centers
                    c = centers(:, k);
                    try
                        [~, ~, zz] = obj.create_sets(Q_vec, c);
                    catch
                        return
                    end

                    occ_cost = 0;
                    try
                        occ = obj.occ_comp.compute_occupancy(zz);
                    catch
                        return
                    end

                    occ_count = 0;
                    for i = 1:obj.occ_comp.num_shapes
                        o = occ{i};
                        [~, o_width,  ~, o_length] = calc_vectors_to_boundary(o);
                        values = [o_width, o_length];
                        for j = 1:length(values)
                            occ_cost = occ_cost + obj.check_occ_size(values(j), obj.max_size_per_occ(i, j));
                            occ_count = occ_count + 1;
                        end
                    end
                    occ_cost = occ_cost / occ_count;
                    acc_occ_cost = acc_occ_cost + occ_cost;
                end
                occ_cost = acc_occ_cost / num_centers;

                [~, z, ~] = obj.create_sets(Q_vec, centers(:, 1));
                ii = interval(z);
                r = rad(ii);
                rad_cost = 0;
                rad_count = 0;
                for i = 1:length(ii)
                    rad_cost = rad_cost + obj.check_rad(r(i), obj.desired_rads_per_dim(i));
                    rad_count = rad_count + 1;
                end
                rad_cost = rad_cost / rad_count;

                cost = occ_cost + rad_cost;
            end

            fun = @obj_fun;
        end

        function cost = check_occ_size(obj, occ_size, max_size)
            cost = 0;
            if occ_size > max_size
                cost = obj.punish_too_large * (occ_size - max_size);
            end
        end

        function cost = check_rad(obj, r, desired)
            cost = 0;
            if r < desired
                cost = (obj.punish_rad_too_small * ((desired - r) / desired))^4;
            end
        end

    end

end
