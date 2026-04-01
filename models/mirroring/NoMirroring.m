classdef NoMirroring < Mirroring
    methods

        function obj = NoMirroring()
            mirror_dim = 0;
            sample_only_greater = true;
            affected_dims = [];
            obj = obj@Mirroring(mirror_dim, sample_only_greater, affected_dims);
        end

    end
end
