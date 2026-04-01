function plot_all_projs(list_of_sets)
    num_dims = dim(list_of_sets{1});
    for i = 1:num_dims
        for j = i + 1:num_dims
            figure;
            hold on;
            for s = list_of_sets
                plot(s{1}, [i, j]);
            end
            axis equal;
            hold off;
        end
    end
end
