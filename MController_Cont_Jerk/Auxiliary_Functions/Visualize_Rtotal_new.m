function void = Visualize_Rtotal_new(R_total)
    colors = struct([]);
    L = struct([]);
    R = struct([]);
    J = struct([]);

    ks_bounds = SamplingSettings(0.1).ks_bounds;

    non_empty_cells = find(~cellfun(@isempty, R_total));
    len = length(non_empty_cells);

    %% Define the color pallette (from blue to red)
    for kk = 1:len
        colors(kk).color = [1 / (len) * (kk - 1), 0, 1 - (kk - 1) * 1 / len];
    end

    tiledlayout("flow");
    for aa = 2:KSTWithTrailerState.STATE_SIZE
        fprintf('%s%d', 'aa = ', aa);
        running_count = 0;
        nexttile;
        hold on;
        for kk = non_empty_cells'
            running_count = running_count + 1;
            L(running_count).plot = plot(R_total{kk}.reachsets, [1 aa], 'Facecolor', colors(running_count).color);
            xlabel(KSTWithTrailerState.REVERSE_STATES(1));
            ylabel(KSTWithTrailerState.REVERSE_STATES(aa));
        end
        running_count = 0;
        for kk = non_empty_cells'
            running_count = running_count + 1;
            R(running_count).plot = plot(R_total{kk}.ref_traj_states(:, 1), R_total{kk}.ref_traj_states(:, aa), 'color', 'black');
        end
        if aa == KSTWithTrailerState.DELTA
            yline([ks_bounds.d_bounds(1), ks_bounds.d_bounds(2)], '--', {'Min', 'Max'});
        end
        if aa == KSTWithTrailerState.V
            yline([ks_bounds.v_bounds(1), ks_bounds.v_bounds(2)], '--', {'Min', 'Max'});
        end
        if aa == KSTWithTrailerState.A
            yline([ks_bounds.a_bounds(1), ks_bounds.a_bounds(2)], '--', {'Min', 'Max'});
        end
    end

    % tiledlayout("flow");
    % for aa = 2:KSTWithTrailerState.STATE_SIZE
    %%    fprintf('%s%d', 'aa = ', aa)
    %    running_count = 0;
    %%    nexttile; hold on;
    %   for kk=non_empty_cells'
    %       running_count = running_count+1
    %       L(running_count).plot = plot(R_total{kk}.reachsets, [1 aa], 'Facecolor', colors(running_count).color)
    % tiledlayout(len,5);
    % nexttile([len, 4]);
    % hold on;
    % for kk = 1:len
    %    L(kk).plot = plot(R_total(kk).RS_ext, [1 2], 'Facecolor', colors(kk).color);
    % end

    % for kk = 1:len
    %    nexttile;
    %    hold on;
    %    R(kk).plot = plot(R_total(kk).R0, [1 2], 'Facecolor', 'w', 'Edgecolor', 'k');
    %    J(kk).plot = plot(R_total(kk).R_endshifted, [1 2], 'Edgecolor', 'g', 'Facecolor', 'y');
    % end

    function NewZono = PlaceZonotope(Zono, place)
        NewZono_center = place;
        NewZono_G = Zono.Z(:, 2:end);
        NewZono = zonotope(NewZono_center, NewZono_G);
    end

end
