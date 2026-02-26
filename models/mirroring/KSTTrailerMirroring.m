classdef KSTTrailerMirroring < Mirroring
    methods

        function obj = KSTTrailerMirroring(state_order)
            mirror_dim = state_order.DELTA;
            sample_only_greater = true;
            affected_dims = [state_order.Y, state_order.PSI, state_order.HA];
            obj = obj@Mirroring(mirror_dim, sample_only_greater, affected_dims);
        end

    end
end
