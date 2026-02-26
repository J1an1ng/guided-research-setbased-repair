classdef LazySampler < handle
    properties (Access = private)
        % cell array where each row contains the values to sample for that
        % dimension
        values = {}
        num_values uint32
        counter (1, :) uint32
    end

    properties (SetAccess = private, GetAccess = public)
        num_samples uint32
    end

    methods (Access = private)

        function vector = current_vector(obj)
            vector = zeros(obj.num_values, 1);
            for k = 1:obj.num_values
                c = obj.counter(k);
                vector(k) = obj.values{k}(c);
            end
        end

    end

    methods

        function obj = LazySampler(values)
            obj.values = values;
            obj.num_values = size(values, 1);
            obj.reset();

            n = 1;
            for i = 1:size(values, 1)
                n = n * size(values{i}, 2);
            end
            obj.num_samples = n;
        end

        function reset(obj)
            obj.counter = ones(1, obj.num_values);
            obj.counter(1, obj.num_values) = 0;
        end

        function next_values = next(obj)
            for i = obj.num_values:-1:1
                obj.counter(i) = obj.counter(i) + 1;
                if obj.counter(i) <= size(obj.values{i}, 2)
                    next_values = obj.current_vector();
                    return
                else
                    obj.counter(i) = 1;
                end
            end
            % if loop above finishes, sampling is done
            obj.reset();
            next_values = [];
        end

    end
end
