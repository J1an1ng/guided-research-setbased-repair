classdef (Abstract) Mirroring < handle & matlab.mixin.Heterogeneous
    properties (Constant)
        MIRROR_CENTER = 0
    end

    properties (GetAccess = public, SetAccess = immutable)
        mirror_dim uint32 % 0 if no mirroring is applied
        sample_only_greater logical
        affected_dims (1, :) uint32
    end

    methods

        function obj = Mirroring(mirror_dim, sample_only_greater, affected_dims)
            obj.mirror_dim = mirror_dim;
            obj.sample_only_greater = sample_only_greater;
            obj.affected_dims = affected_dims;
        end

        function [r, must_mirror] = mirror(obj, r)
            if obj.mirror_dim == 0
                must_mirror = false;
                return
            end

            mirror_dim_itv = project(r, obj.mirror_dim);
            must_mirror = (center(mirror_dim_itv) > obj.MIRROR_CENTER) ~= obj.sample_only_greater;
            if must_mirror
                r = mirror_set(r, [obj.mirror_dim, obj.affected_dims]);
            end
        end

    end
end
