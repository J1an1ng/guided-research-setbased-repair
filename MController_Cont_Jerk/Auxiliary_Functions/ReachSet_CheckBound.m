function [boolout, fail_at] = ReachSet_CheckBound(R_total)
    % REACHSET_CHECKBOUND Summary of this function goes here
    %   Detailed explanation goes here
    non_empty_cells = find(~cellfun(@isempty, R_total));
    complete_cumulative = true;
    ks_bounds = SamplingSettings(0.1).ks_bounds;
    ks_bounds.a_bounds(2) = 2.0;
    fail_at = {'ReachSet', 'Over Bounds Step'};
    for kk = non_empty_cells'
        cumulative = true;
        for jj = 1:length(R_total{kk}.reachsets)
            cumulative = cumulative & Check_BoundaryCond(R_total{kk}.reachsets(jj), ks_bounds, [3, 7]);
        end
        if ~cumulative
            fail_at{1, end + 1} = [kk, jj];
        end
        complete_cumulative = complete_cumulative & cumulative;
    end

    boolout = complete_cumulative;

end
