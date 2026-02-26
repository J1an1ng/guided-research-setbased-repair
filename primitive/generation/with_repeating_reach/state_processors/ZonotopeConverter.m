classdef ZonotopeConverter < handle
    properties (Constant)
        SET_TYPES = ["interval", "zonotope", "conZonotope"] % , "polyZonotope"]
    end

    methods (Static)

        function res = convert_to_zono(cora_set)
            assert (any(arrayfun(@(s) isa(cora_set, s), ZonotopeConverter.SET_TYPES)), ...
                'The set type is not supported by the state processor:%s.', ...
                class(cora_set));
            res = cora_set;
            if isa(res, 'interval')
                res = zonotope(res);
                return
            end
            % Zonotope and conZonotope stay the same
        end

    end
end
