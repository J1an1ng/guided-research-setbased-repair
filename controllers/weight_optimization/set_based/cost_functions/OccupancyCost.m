classdef OccupancyCost < CostDef
    properties (Access = private)
        settings PrimitiveGenerationSettings
        occ_comp OccupancyComputer
        freq uint32
        width_length_weight (:, 1) double
    end

    methods

        function obj = OccupancyCost(settings, occ_comp, freq, width_length_weight)
            obj.settings = settings;
            obj.occ_comp = occ_comp;
            obj.freq = freq;
            obj.width_length_weight = width_length_weight;
        end

        function cost = evaluate(obj, ~, reach_res)
            w = obj.settings.params.truck_width;
            l = obj.settings.params.truck_length;
            % trailer_l = obj.settings.params.trailer_l;
            [max_width, max_length] = compute_occupancy(reach_res.reachsets, obj.occ_comp, obj.freq, w, l);
            % max_width = max_width - w;
            % max_length = max_length - trailer_l;
            width_cost = map_width_cost(max_width);
            length_cost = map_length_cost(max_length);
            cost = obj.width_length_weight(1) * width_cost + obj.width_length_weight(2) * length_cost;
        end

    end
end

function [max_width, max_length] = compute_occupancy(reachsets, occ_computer, freq, veh_width, veh_length)
    max_width = -inf;
    max_length = -inf;
    r_idcs = 1:freq:length(reachsets);
    for k = 1:length(r_idcs)
        r = reachsets(r_idcs(k));
        occs = occ_computer.compute_accumulated_occupancy(r.timeInterval.set);

        for j = 1:length(occs)
            o = occs{j};
            idcs = 1:size(o.set.Vertices, 1) - 1;
            dists = arrayfun(@(i) norm(o.set.Vertices(i, :) - o.set.Vertices(i + 1, :)), idcs);
            occ_width = min(dists(dists > veh_width));
            occ_length = max(dists(dists > veh_length));
            max_width = max(max_width, occ_width);
            max_length = max(max_length, occ_length);
        end
    end
end

function cost = map_width_cost(width_cost)
    cost = width_cost^6;
end

function cost = map_length_cost(length_cost)
    cost = length_cost^2;
end
