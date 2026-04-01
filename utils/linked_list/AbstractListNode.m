classdef (Abstract) AbstractListNode < handle
    properties (Abstract, Constant)
        is_leaf logical
    end

    methods (Abstract)
        acc = reduce(obj, fun, acc)
        acc = recursive_reduce(obj, fun, acc)
    end

    methods

        function arr = to_array(obj)
            current = obj;
            n = obj.count;
            arr = cell(n, 1);
            for i = 1:n
                arr{i} = current.value;
                current = current.succ;
            end
        end

        function n = count(obj)
            current = obj;
            n = 0;
            while ~current.is_leaf
                n = n + 1;
                current = current.succ;
            end
        end

    end
end
