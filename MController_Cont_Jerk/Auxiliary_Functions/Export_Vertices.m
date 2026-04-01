function void = Export_vertices(RSet)
    % Export vertices of the reachset
    % loop over all individual Reachsets
    % (corresponding to 'Constant Input Intervals')
    fid = fopen('Vertices', 'wt');

    % Loop over all reachsteps (in the Constant Input Interval
    for i = 1:length(RSet)
        % Get information on the times in this Constant Input Interval
        times = [];
        for k = 1:length(RSet(i, 1).timePoint.time)
            times(end + 1) = RSet(i, 1).timePoint.time{k, 1};
        end
        fprintf(fid, '%f,', min(times));
        fprintf(fid, '\t');
        for j = 1:length(RSet(i, 1).timePoint.set)
            % project Zonotope onto the trajectory following dimensions
            % i.e. cut away the nominal input dimensions
            Z = project(RSet(i, 1).timePoint.set{j, 1}, [1:6]);
            % Reduce zonotope order of Z
            Zr = reduce(Z, 'cluster', 2);
            % Extract the vertices from reduced Zonotope
            V = Zr.vertices;
            for k = 1:size(V, 2)
                fprintf(fid, '[');
                fprintf(fid, '%f;', V(1:end - 1, k));
                fprintf(fid, '%f', V(end, k));
                fprintf(fid, '],');
            end
            fprintf(fid, '\t');
            fprintf(fid, '%f', max(times));
            fprintf(fid, '\n\n');
        end
    end

end
