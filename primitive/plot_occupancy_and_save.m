function plot_occupancy_and_save(settings, p, R_primitive, freq)
    % plot occupancy of the primitive along the reference
    % trajectory
    occ_fig_out_path = strcat(settings.result_dir.fig_out_path, "/occ-", p.json, '.fig');
    f = figure;

    plot_occupancy(settings, R_primitive, freq);

    if settings.save_plot
        saveas(f, occ_fig_out_path, 'fig');
    end

    if ~settings.show_plot
        close(f);
    end
end
