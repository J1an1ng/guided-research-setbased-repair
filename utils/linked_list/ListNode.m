classdef ListNode < AbstractListNode
    properties (Constant)
        is_leaf = 0
    end

    properties
        item
        succ AbstractListNode = ListLeaf()
    end

    methods

        function obj = ListNode(item, succ)
            obj.item = item;
            obj.succ = succ;
        end

        function acc = reduce(obj, fun, acc)
            acc = fun(acc, obj.item);
            acc = obj.succ.reduce(fun, acc);
        end

        function acc = recursive_reduce(obj, fun, acc)
            el = obj.item;
            if isa(el, 'AbstractListNode')
                el = el.recursive_reduce(fun, []);
            end
            acc = fun(acc, el);
            acc = obj.succ.recursive_reduce(fun, acc);
        end

    end
end
