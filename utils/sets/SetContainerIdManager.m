classdef SetContainerIdManager < handle
    properties (Access = private)
        next_id = 1
    end

    methods

        function id = gen_next_id(obj)
            id = obj.next_id;
            obj.next_id = obj.next_id + 1;
        end

        function id = get_current_id(obj)
            id = obj.next_id - 1;
        end

    end
end
