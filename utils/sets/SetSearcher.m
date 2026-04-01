classdef SetSearcher < handle
    properties (Access = private)
        dims (1, :) uint32
        idx_to_set_key dictionary = dictionary()
        searcher
        get_scales
        radius_scale double
    end

    methods

        function obj = SetSearcher(sets, dims, get_scales, radius_scale_ext)
            obj.dims = dims;
            obj.get_scales = get_scales;
            obj.radius_scale = 1.0 + radius_scale_ext;
            [obj.idx_to_set_key, obj.searcher] = create_searcher(sets, dims);
        end

        function res = search(obj, set)
            scales_1 = obj.get_scales(set.c);
            assert (length(scales_1) == length(obj.dims));
            scales_2 = rad(project(set.enclosing_itv, obj.dims));
            scales = max(scales_1, scales_2);

            % we want to find all sets whose center is within the
            % maximum interval of the given set (represented by scales_1)
            % or which is within the bounding box of the given set (scaled a bit, as
            % we allow a certain growth when merging sets)
            r = sqrt(obj.radius_scale * obj.radius_scale * length(obj.dims));
            idcs = rangesearch(obj.searcher, set.c(obj.dims)', r, ...
                'Distance', 'seuclidean', 'Scale', scales);
            idcs = idcs{1};
            res = obj.idx_to_set_key(idcs);
        end

    end
end

function [idx_to_set_key, searcher] = create_searcher(sets, dims)
    values = sets.values();
    idx_to_set_key = dictionary(1:length(values), sets.keys()');
    if ~isempty(values)
        centers = arrayfun(@(i) values(i).c(dims), 1:length(values), 'UniformOutput', false);
        centers = cell2mat(centers);
        centers = centers';
    else
        centers = double.empty(0, length(dims));
    end
    searcher = ExhaustiveSearcher(centers);
end
