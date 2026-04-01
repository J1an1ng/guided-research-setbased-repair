classdef ListLeaf < AbstractListNode
    properties (Constant)
        is_leaf = 1
    end

    methods

        function acc = reduce(~, ~, acc)
        end

        function acc = recursive_reduce(~, ~, acc)
        end

    end
end
