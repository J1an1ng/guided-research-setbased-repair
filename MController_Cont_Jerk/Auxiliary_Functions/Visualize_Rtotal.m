function void = Visualize_Rtotal(R_total)
    colors = struct([]);
    L = struct([]);
    R = struct([]);
    J = struct([]);

    len = length(R_total);

    %% Define the color pallette (from blue to red)
    for kk = 1:len
        colors(kk).color = [1 / (len) * (kk - 1), 0, 1 - (kk - 1) * 1 / len];
    end

    tiledlayout(len, 5);
    nexttile([len, 4]);
    hold on;
    for kk = 1:len
        L(kk).plot = plot(R_total(kk).RS_ext, [1 2], 'Facecolor', colors(kk).color);
    end

    for kk = 1:len
        nexttile;
        hold on;
        R(kk).plot = plot(R_total(kk).R0, [1 2], 'Facecolor', 'w', 'Edgecolor', 'k');
        J(kk).plot = plot(R_total(kk).R_endshifted, [1 2], 'Edgecolor', 'g', 'Facecolor', 'y');
    end

    function NewZono = PlaceZonotope(Zono, place)
        NewZono_center = place;
        NewZono_G = Zono.Z(:, 2:end);
        NewZono = zonotope(NewZono_center, NewZono_G);
    end

end
