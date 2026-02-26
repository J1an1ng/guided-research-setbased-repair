f = load("out/dummy_3/reach_results/primitives/final_sets/100004.mat", "p_final_set");
display(f);

subset_rel_file = 'out/dummy_3/subset_relation.json';
fid = fopen(subset_rel_file);
raw = fread(fid, inf);
str = char(raw');
fclose(fid);
subset_rel = jsondecode(str);

fn = fieldnames(subset_rel);
c = 0;
t = 0;
for k = 1:numel(fn)
    t = t + 1;
    v = subset_rel.(fn{k});
    if v == '0'
        c = c + 1;
    end
end

display(c);
display(t);
