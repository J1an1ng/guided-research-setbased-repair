% Quick diagnosis for subset_relation coverage
% Run from matlab_primitive_generation or add path accordingly.

out_dir = fullfile('out', 'primitives_for_edgar', 'v2');
checkpoint = fullfile(out_dir, 'internal', 'checkpoint_for_BoxLikeStateSpaceDividerWithCorrelations_1.mat');
final_sets_dir = fullfile(out_dir, 'internal', 'primitives_final_sets');

if ~isfile(checkpoint)
    error('Checkpoint not found: %s', checkpoint);
end

load(checkpoint, 'dump_obj');
if ~isfield(dump_obj, 'current_batch')
    error('Checkpoint missing current_batch.');
end

current_batch = dump_obj.current_batch;
num_sets = numel(current_batch);
fprintf('Initial sets in batch: %d\n', num_sets);

centers = cellfun(@(x) x.c, current_batch, 'UniformOutput', false);
centers = [centers{:}];

files = dir(fullfile(final_sets_dir, '*.mat'));
if isempty(files)
    error('No final set files found in %s', final_sets_dir);
end

sample_n = min(20, numel(files));
idx = round(linspace(1, numel(files), sample_n));

hit_count = 0;
for k = 1:sample_n
    load(fullfile(final_sets_dir, files(idx(k)).name), 'p_final_set');
    if isempty(p_final_set)
        fprintf('Final set missing in %s\n', files(idx(k)).name);
        continue
    end

    found = false;
    for j = 1:num_sets
        if current_batch{j}.cora_set.contains(0.999 * p_final_set.cora_set)
            found = true;
            break
        end
    end

    if found
        hit_count = hit_count + 1;
    end
end

fprintf('Sampled final sets: %d\n', sample_n);
fprintf('Containment hits: %d\n', hit_count);

% Optional: nearest-center distance stats (rough indicator)
final_centers = zeros(size(centers, 1), sample_n);
for k = 1:sample_n
    load(fullfile(final_sets_dir, files(idx(k)).name), 'p_final_set');
    final_centers(:, k) = p_final_set.c;
end

dists = zeros(sample_n, 1);
for k = 1:sample_n
    diffs = centers - final_centers(:, k);
    dists(k) = min(vecnorm(diffs));
end

fprintf('Min center distance (mean): %.4f\n', mean(dists));
fprintf('Min center distance (max): %.4f\n', max(dists));

% Compare final set sizes to nearest initial set sizes
size_ratios = zeros(sample_n, 1);
size_exceeds = false(sample_n, 1);
for k = 1:sample_n
    load(fullfile(final_sets_dir, files(idx(k)).name), 'p_final_set');
    final_rad = rad(p_final_set.enclosing_itv);
    diffs = centers - p_final_set.c;
    [~, nearest_idx] = min(vecnorm(diffs));
    initial_rad = rad(current_batch{nearest_idx}.enclosing_itv);

    ratio = ones(size(final_rad));
    zero_mask = initial_rad == 0;
    ratio(~zero_mask) = final_rad(~zero_mask) ./ initial_rad(~zero_mask);
    ratio(zero_mask & final_rad > 0) = inf;

    size_ratios(k) = max(ratio);
    size_exceeds(k) = size_ratios(k) > 1.0;
end

fprintf('Max size ratio (mean): %.4f\n', mean(size_ratios));
fprintf('Max size ratio (max): %.4f\n', max(size_ratios));
fprintf('Size exceeds count: %d/%d\n', sum(size_exceeds), sample_n);
