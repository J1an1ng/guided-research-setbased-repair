classdef InputOutOfBoundsException < MException

    properties (Constant)
        ID = "CONTROL:input_violates_bounds"
    end

    properties
        bound_violations (:, 1) double
    end

    methods

        function obj = InputOutOfBoundsException(bound_violations)
            obj@MException(InputOutOfBoundsException.ID, 'Input violates bounds');
            obj.bound_violations = bound_violations;
        end

    end
end
