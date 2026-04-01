function sets = prepare_for_export(sets, num_workers)
    if num_workers == 1
        for i = 1:length(sets)
            sets{i}.compute_cora_set_for_export();
        end
    else
        parfor (i = 1:length(sets), num_workers)
            % copying and re-assigning is necessary because matlab sucks:
            % https://de.mathworks.com/matlabcentral/answers/26604-assigning-handle-classes-in-a-parfor-loop
            % this is also the reason why we need to return sets
            s = sets{i};
            s.compute_cora_set_for_export();
            sets{i} = s;
        end
    end
end
