function plot_trajectories(init_interval, vehicle_plotter, traj_gen, k, plot_discarded, out_dir, idx)
    % arguments
    %     init_interval interval
    %     vehicle_plotter VehiclePlotter
    %     traj_gen ValidInpTrajGenerator
    %     k AbstractStateOrder
    %     plot_discarded logical
    %     out_dir string
    %     idx uint32
    % end

    init_state = center(init_interval);

    if exist("out_dir", "var")
        save_plot = true;
        assert(exist("idx", "var"));
        figure('visible', 'off');
        out_file = strcat(out_dir, "/", string(idx), "_", strjoin(string(init_state), " , "), "%s.png");
    else
        save_plot = false;
        figure;
    end

    hold on;
    axis equal;

    vehicle_plotter.plot_vehicle(init_state, 'r');

    [trajectories, discarded_trajs] = traj_gen.generate(center(init_interval));

    fprintf(['#Ref trajs: %d.\n ' ...
             '#Discarded trajs: %d.\n'], ...
             length(trajectories), length(discarded_trajs));

    traj_plots = plot_all_traj(trajectories, @parula, k, vehicle_plotter);
    if save_plot
        export_plot(sprintf(out_file, "_valid"));
    end

    if plot_discarded
        %% make invalid trajectories gray
        traj_plots = vertcat(traj_plots{:});
        for i = 1:length(traj_plots)
            make_transparent(traj_plots{i}, 0.2);
        end

        plot_all_traj(discarded_trajs, @gray, k, vehicle_plotter);
        if save_plot
            export_plot(sprintf(out_file, "_discarded"));
        end
    end

    hold off;
end

function make_transparent(p, alpha)
    if isprop(p, "MarkerFaceAlpha")
        p.MarkerFaceAlpha = alpha;
    else
        p.Color = [p.Color alpha];
    end
end

function export_plot(file_name)
    axis 'auto x';
    axis 'auto y';
    axis padded;
    exportgraphics(gcf, file_name, 'Resolution', 300);
end

function traj_plots = plot_all_traj(trajs, color_map, k, vehicle_plotter)
    n = length(trajs);
    c_map = color_map(n);

    traj_plots = cell(n, 1);
    for t = 1:n
        p = plot_single_traj(trajs{t}, c_map(t, :), k, vehicle_plotter);
        traj_plots{t} = p;
    end
end
