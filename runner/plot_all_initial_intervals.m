%% Model settings
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;

%% IMPORTANT: disable CORA arg checking by adding the folder disable_cora_arg_checks
%% to MATLAB path.

%% Configs
params = create_kst_trailer_params_1();
constraints = KSTTrailerConstraints(k, i, params);
% constraints = RestrictiveKSTTrailerConstraints(k, i, params, 0.22, 0.35);

% interval_sampler = create_medium_coarse_initial_interval_sampler_for_kst_trailer( ...
% k, params, constraints);
% interval_sampler = create_initial_interval_sampler_for_kst_trailer_1( ...
%     k, params, constraints);
interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr(k, params, constraints);

out_dir = "out/interval_sampling";
mkdir(out_dir);

interval_file = out_dir + "/intervals.mat";

if exist(interval_file, 'file')
    fprintf("File found\n");
    load(interval_file, 'init_intervals');
else
    fprintf("File not found. Generate...\n");
    [init_intervals, num_discarded] = interval_sampler.sample_all();
    fprintf("#Intervals: %d \n. #Discarded: %d \n.", length(init_intervals), num_discarded);
    save(interval_file, 'init_intervals');
end

dimensions = [k.V k.DELTA; k.V k.HA];
file_names = ["v-delta"; "v-ha"];

for d = 1:length(dimensions)
    dims = dimensions(d, :);
    figure;

    vs = zeros(4 * length(init_intervals), 2);
    for i = 1:length(init_intervals)
        w = vertices(to_red_interval(init_intervals(i, :, :), dims));
        start = (i - 1) * 4 + 1;
        vs(start:start + 3, :) = w';
    end
    [k, av] = convhull(vs);
    plot(vs(k, 1), vs(k, 2));
    axis 'auto x';
    axis 'auto y';
    axis padded;
    exportgraphics(gcf, out_dir + "/" + file_names(d) + ".png", 'Resolution', 300);
end

function res = to_red_interval(arr, sampled_dims)
    res = interval(squeeze(arr(1, sampled_dims, 1))', squeeze(arr(1, sampled_dims, 2))');
end
