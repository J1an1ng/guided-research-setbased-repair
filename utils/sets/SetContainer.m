classdef SetContainer < handle & JsonEncodable
    properties (Constant)
        base_type = 'interval'
        id_properties = ["inner_base_set"]
    end

    properties (GetAccess = public, SetAccess = immutable)
        id uint32
        cora_set % interval, zonotope, conZonotope
        c (:, 1)

        is_base logical

        enclosing_itv

        inner_base_set
        has_inner_base_set logical
    end

    methods

        function obj = SetContainer(id, cora_set, is_base, inner_base_set)
            if ~exist('inner_base_set', 'var')
                inner_base_set = [];
            end
            [inner_base_set, has_inner_base_set] = validate_args( ...
                obj.base_type, cora_set, is_base, inner_base_set);

            obj.id = id;
            obj.cora_set = cora_set;
            obj.c = center(cora_set);
            obj.is_base = is_base;
            obj.enclosing_itv = interval(cora_set);
            obj.inner_base_set = inner_base_set;
            obj.has_inner_base_set = has_inner_base_set;
        end

    end
end

function [inner_base_set, has_inner_base_set] = validate_args(base_type, cora_set, is_base, inner_base_set)
    has_inner_base_set = ~is_empty_var(inner_base_set);
    if is_base
        assert(isa(cora_set, base_type), ...
            'SetContainer:InvalidInput', ...
            'Base set must be a %', base_type);
        assert(~has_inner_base_set, ...
            'SetContainer:InvalidInput', ...
            'Base set cannot have an inner base set');
        inner_base_set = cora_set;
        has_inner_base_set = true;
    else
        if ~has_inner_base_set
            inner_base_set = [];
        end
        assert(~has_inner_base_set || isa(inner_base_set, base_type), ...
            'SetContainer:InvalidInput', ...
            'Inner base set must be an %s', base_type);
    end
end
