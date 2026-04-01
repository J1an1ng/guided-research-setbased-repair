classdef (Abstract) JsonEncodable < handle
    % To make a class C json-encodable, let C inherit from JsonEncodable
    % and overwrite the constant property :id_properties below.
    properties (Abstract, Constant)
        % List of properties whose values should be used for json-encoding
        id_properties (:, 1) string
    end

    properties (Dependent)
        json
    end

    methods

        function value = get.json(obj)
            value = jsonencode(obj);
        end

    end

    methods (Access = public)

        function json = jsonencode(obj, varargin)
            result = [];
            num_props = length(obj.id_properties);
            if num_props == 1
                % we squeeze the chain if there is only one property
                p = obj.id_properties(1);
                result = obj.(p);
            else
                for i = 1:num_props
                    p = obj.id_properties(i);
                    prop_value = obj.(p);
                    result.(p) = prop_value;
                end
            end
            json = jsonencode(result, varargin{:});
        end

        function h = keyHash(obj) % mh:ignore_style
            % Idea to use json as hash is from
            % https://de.mathworks.com/matlabcentral/answers/521167-is-
            % there-an-efficient-way-to-calculate-hash-for-objects#answer_428727
            if ~isempty(obj)
                %% Important: according to MATLAB documentation, the
                %% built-in keyHash function uses a random seed and can
                %% therefore differ between MATLAB sessions and parallel workers!
                %% Thus, when we dump primitives to .mat-files, we cannot
                %% do this directly within dictionaries, as their
                %% hash-value might change between sessions, thus looking
                %% up an already existing primitive would not be successfull
                %% in general!!
                h = keyHash(obj.json);
            else
                h = 0;
            end
        end

        function tf = keyMatch(objA, objB) % mh:ignore_style
            tf = keyMatch(objA.json, objB.json);
        end

    end
end
