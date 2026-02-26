classdef (Abstract) VehiclePlotter < handle
    properties (Access = protected)
        occ_computer OccupancyComputer
        k AbstractStateOrder

        pos_dims (1, 2) uint32
    end

    methods

        function obj = VehiclePlotter(occ_computer, k)
            obj.occ_computer = occ_computer;
            obj.k = k;
            obj.pos_dims = [k.X k.Y];
        end

    end

    methods

        function res = plot_vehicle(obj, state, color, ax)
            occs = obj.occ_computer.compute_occupancy(interval(state, state));
            res = cell(length(occs), 1);
            for i = 1:length(occs)
                p = plot(occs{i}, obj.pos_dims, 'Color', color, 'Parent', ax);
                hold on;
                res{i} = p;
            end
        end

    end
end
