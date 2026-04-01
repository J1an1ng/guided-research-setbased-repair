classdef OccupancyComputer < handle & matlab.mixin.Heterogeneous
    properties (Access = protected)
        occ_comps (:, 1) SingleComponentOccupancyComputer
    end

    properties (Dependent)
        num_shapes uint32
    end

    methods

        function value = get.num_shapes(obj)
            value = length(obj.occ_comps);
        end

    end

    methods

        function obj = OccupancyComputer(occ_comps)
            obj.occ_comps = occ_comps;
        end

        function res = compute_occupancy(obj, reachset)
            res = cell(obj.num_shapes, 1);
            for o = 1:obj.num_shapes
                occ = obj.occ_comps(o).compute_occ(reachset);
                %% TODO maybe reduce order of occupancy-zonotopes again if there are
                % too many vertices
                V = vertices(occ);
                % Otherwise a warning about the performed simplification is raised
                w = warning();
                warning('off');
                res{o, 1} = polygon(V(1, :), V(2, :));
            end
            warning(w);
        end

        function acc = compute_accumulated_occupancy(obj, reachsets)
            num_reachsets = length(reachsets);

            R = cell(num_reachsets, obj.num_shapes);

            for i = 1:num_reachsets
                R(i, :) = obj.compute_occupancy(reachsets{i});
            end

            acc = cell(obj.num_shapes, 1);
            for k = 1:obj.num_shapes
                acc{k} = R{1, k};
            end

            for i = 2:num_reachsets
                for k = 1:obj.num_shapes
                    acc{k} = acc{k} | R{i, k};
                end
            end
        end

        function merged_occs = compute_accumulated_occupancy_for_multiple_time_intervals(obj, ...
            reachsets, start_idx, end_idx)
            occs = arrayfun(@(i) obj.compute_accumulated_occupancy(reachsets(i).timeInterval.set), ...
              start_idx:end_idx, 'UniformOutput', false);
            merged_occs = occs{1};
            for j = 2:length(occs)
                for k = 1:obj.num_shapes
                    merged_occs{k} = merged_occs{k} | occs{j}{k};
                end
            end
        end

        function res = compute_occupancy_for_single_state(obj, s)
            res = cell(obj.num_shapes, 1);
            for o = 1:obj.num_shapes
                occ_v = obj.occ_comps(o).compute_occ_for_single_state(s);
                res{o, 1} = polygon(occ_v(1, :), occ_v(2, :));
            end
        end

    end
end
