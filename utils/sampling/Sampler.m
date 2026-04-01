classdef Sampler < handle
    properties (Access = private)
        % cell array where each row contains the values to sample for that
        % dimension
        values = {}
        num_values uint32
    end

    properties (Access = public)
        num_samples uint32
    end

    methods

        function obj = Sampler(values)
            obj.values = values;
            obj.num_values = size(values, 1);

            n = 1;
            for i = 1:size(values, 1)
                n = n * size(values{i}, 2);
            end
            obj.num_samples = n;
        end

        function all_values = sample_all(obj)
            lazy_sampler = LazySampler(obj.values);
            all_values = cell(obj.num_samples, 1);
            i = 1;
            next_value = lazy_sampler.next();
            while ~isempty(next_value)
                all_values{i} = next_value;
                i = i + 1;
                next_value = lazy_sampler.next();
            end
        end

    end
end
