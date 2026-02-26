classdef (Abstract) AbstractStateOrder < handle & matlab.mixin.Heterogeneous
    % Implement this class to define for a certain model,
    % how the dimensions of a state vector are ordered, e.g. at which dimension
    % the velocity is placed.
    % Hint: inheriting from matlab.mixin.Heterogeneous prevents that we need to
    % define a default concrete class for this abstract class when using the
    % abstract class as type validator.

    properties (Constant)
        non_state_dim_props = dictionary(["non_state_dim_props", "SIZE"], ones(1, 2))
    end

    methods (Static, Abstract)
        size = get_size()
    end

    methods (Abstract)
        label = label_from_state_idx(obj, idx)
        unit_string = unit_string_from_state_idx(obj, idx)
    end

    methods

        function idcs = create_state_order_mapping(obj, other_state_order)
            idcs = zeros(1, obj.get_size());
            props = properties(obj);
            for i_prop = 1:length(props)
                prop_name = props{i_prop};
                if ~isKey(obj.non_state_dim_props, prop_name)
                    idcs(obj.(prop_name)) = other_state_order.(prop_name);
                end
            end
        end

        function idcs  = revert_state_order_mapping(obj, state_order_mapping)
            idcs = zeros(1, obj.get_size());
            for i = 1:obj.get_size()
                idcs(i) = find(state_order_mapping == i);
            end
        end

    end
end
