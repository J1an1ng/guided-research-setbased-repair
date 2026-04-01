classdef KSTTrailerInpDeltaMirroring < Mirroring
    methods

        function obj = KSTTrailerInpDeltaMirroring(state_order)
            mirror_dim = state_order.HA;
            sample_only_greater = true;
            affected_dims = [state_order.Y, state_order.PSI];
            obj = obj@Mirroring(mirror_dim, sample_only_greater, affected_dims);
        end

    end
end
