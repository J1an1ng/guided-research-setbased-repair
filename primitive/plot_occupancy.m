function plot_occupancy(settings, R_primitive, freq)
    color_map_truck = @sky;
    color_map_trailer = @copper;

    idcs = freq:freq:length(R_primitive);

    n = length(idcs);
    c_map_truck = color_map_truck(n);
    c_map_trailer = color_map_trailer(n);

    for k = 1:length(idcs)
        r = R_primitive(idcs(k));
        occs = settings.occ_comp.compute_accumulated_occupancy(r.timeInterval.set);
        c_truck = c_map_truck(k, :);
        c_trailer = c_map_trailer(k, :);
        colors = {c_truck, c_trailer};

        for j = 1:length(occs)
            plot(occs{j}, [1 2], 'color', colors{j}, 'LineWidth', 2);
            axis equal;
            hold on;
        end
    end
end
