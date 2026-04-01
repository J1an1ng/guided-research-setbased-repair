classdef ListOf2Ds < handle
    %% Data structure for storing an initially unknown number of 2D-arrays
    %% Hint: cell arrays are extremely slow when they grow
    properties (Access = private)
        dims (1, 2) uint32
        list_root AbstractListNode = ListLeaf()
        max_arr_size = 200000
        current_idx uint32 = 1
    end

    methods

        function obj = ListOf2Ds(dims)
            obj.dims = dims;
            obj.create_new_list_node();
        end

        function add(obj, item)
            if obj.current_idx > length(obj.list_root.item)
                obj.create_new_list_node();
            end
            obj.list_root.item(obj.current_idx, :, :) = item;
            obj.current_idx = obj.current_idx + 1;
        end

        function res = to_arr(obj)
            n = obj.list_root.count;
            num_entries = max(0, n - 1) * obj.max_arr_size + obj.current_idx - 1;
            res = zeros([num_entries, obj.dims]);
            i = size(res, 1);

            n_current = obj.current_idx - 1;
            res(i - n_current + 1:i, :, :) = obj.list_root.item(1:obj.current_idx - 1, :, :);
            i = i - n_current;

            current = obj.list_root.succ;
            while ~current.is_leaf
                n_current = length(current.item);
                res(i - n_current + 1:i, :, :) = current.item;
                i = i - n_current;

                current = current.succ;
            end
        end

    end

    methods (Access = private)

        function create_new_list_node(obj)
            obj.list_root = ListNode( ...
                zeros([obj.max_arr_size, obj.dims]), ...
                obj.list_root);
            obj.current_idx = 1;
        end

    end
end
