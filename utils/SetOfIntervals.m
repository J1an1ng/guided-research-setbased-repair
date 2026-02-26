classdef SetOfIntervals < handle
    properties (Access = private)
        intervals (:, 2, :) double
        k_d_tree = []
    end

    methods

        function obj = SetOfIntervals(num_dims)
            obj.intervals = double.empty(num_dims, 2, 0);
            obj.recreate_tree();
        end

        function res = num_itvs(obj)
            res = size(obj.intervals, 3);
        end

        function recreate_tree(obj)
            if obj.num_itvs() > 0
                centers = arrayfun(@(i) mean(squeeze(obj.intervals(:, :, i)), 2), ...
                    1:obj.num_itvs(), 'UniformOutput', false);
                centers = cell2mat(centers);
                centers = centers';
                obj.k_d_tree = ExhaustiveSearcher(centers);
            end
        end

        function res = add(obj, itv)
            res = obj.exists(itv);
            if ~res
                obj.intervals(:, :, end + 1) = itv;
                obj.recreate_tree();
            end
        end

        function res = exists(obj, itv)
            if obj.num_itvs() == 0
                res = false;
                return
            end
            c = mean(itv, 2);
            nns = knnsearch(obj.k_d_tree, c', 'K', 3);
            itvs = obj.intervals(:, :, nns);
            equals = arrayfun(@(i) isequal(itv, itvs(:, :, i)), 1:size(itvs, 3));
            res = any(equals);
        end

    end
end
