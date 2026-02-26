config.delta_size = 0.15;
config.max_delta = 0.45 - config.delta_size;
config.delta_overlap = 0.01;
config.ha_size = 0.2;
config.max_ha = 0.2; % 0.3;
config.ha_overlap = 0.01;
config.v_size = 0.5;
config.v_overlap = 0.1;

config.start_v = 22.22 - config.v_size / 2;
config.min_v = 20.0;

% Base intervals on top level
itvs = generate_itvs_at_speed(config, config.start_v, false, false);

figure;
plot_itvs(itvs);
set_plot_config(config);

% Potential successor intervals:
succ_v = config.start_v - 1.0;
succ_itvs_smaller_1 = generate_itvs_at_speed(config, succ_v, true, true);
succ_itvs_smaller_2 = generate_itvs_at_speed(config, succ_v, true, true);
succ_itvs_smaller_3 = generate_itvs_at_speed(config, succ_v, true, true);
succ_itvs_larger_1 = generate_itvs_at_speed(config, succ_v, true, false);

figure;
plot_itvs(succ_itvs_smaller_1);
plot_itvs(succ_itvs_smaller_2);
plot_itvs(succ_itvs_smaller_3);
plot_itvs(succ_itvs_larger_1);
set_plot_config(config);

% New starting intervals:
new_start_itvs = {};
for i = 1:length(itvs)
    succ_itv_1 = succ_itvs_smaller_1{i};
    succ_itv_2 = succ_itvs_smaller_2{i};
    succ_itv_3 = succ_itvs_smaller_3{i};
    succ_itv_4 = succ_itvs_larger_1{i};
    new_itv = succ_itv_1 | succ_itv_2 | succ_itv_3 | succ_itv_4;
    new_start_itvs{end + 1} = new_itv;
end

figure;
plot_itvs(new_start_itvs);
set_plot_config(config);

function set_plot_config(config)
    padding = 0.05;
    xlim([-padding config.max_delta + padding]);
    ylim([-config.ha_size - padding config.ha_size + padding]);
    zlim([config.min_v config.start_v + config.v_size]);

    fontsize = 22;

    xlabel('{\delta} (rad)', 'FontSize', fontsize);
    ylabel('{\alpha} (rad)', 'FontSize', fontsize);
    zlabel('v (m/s)', 'FontSize', fontsize);

    ax = gca;
    ax.FontSize = fontsize;
end

function itvs = generate_itvs_at_speed(config, v, random_size, random_smaller)
    itvs = {};
    itv_rad = [config.delta_size / 2 + config.delta_overlap; config.ha_size / 2 + config.ha_overlap; config.v_size / 2 + config.v_overlap];
    for delta = config.delta_size / 2:config.delta_size:config.max_delta - config.delta_size / 2
        for ha = -config.max_ha + config.ha_size / 2:config.ha_size:config.max_ha - config.ha_size / 2
            c = [delta; ha; v];
            current_itv_rad = itv_rad;

            if random_size
                if random_smaller
                    rand_factor = 0.3 * rand();
                    current_itv_rad = current_itv_rad * (1 - rand_factor);
                else
                    rand_factor = 0.1 * rand();
                    current_itv_rad = current_itv_rad * (1 + rand_factor);
                end
            end
            lb = c - current_itv_rad;
            ub = c + current_itv_rad;
            itv = interval(lb, ub);
            itvs{end + 1} = itv;
        end
    end
end

function plot_itvs(itvs)
    colors = lines(length(itvs)); % Get a colormap with enough unique colors

    for i = 1:length(itvs)
        itv = itvs{i};
        verts = vertices(itv);
        xs = verts(1, :)';
        ys = verts(2, :)';
        zs = verts(3, :)';
        sh = alphaShape(xs, ys, zs);

        color_i = colors(i, :);

        p = plot(sh, 'EdgeColor', 'none', 'FaceColor', color_i, 'FaceAlpha', 0.6); % , 'FaceColor', color_i, 'FaceAlpha', 0.6); % Face in unique color
        hold on;
        plot3(xs, ys, zs, 'o', 'MarkerSize', 3, 'Color', color_i, 'MarkerFaceColor', color_i); % Vertices in same color
    end
end
