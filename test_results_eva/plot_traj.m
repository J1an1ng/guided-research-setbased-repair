file_path = '/home/sebastian/Documents/Primitives/TestResults/ee3f68fb-da96-4002-8c8b-5b9183a4400a.csv';

t_col = "header_timestamp";
opts = detectImportOptions(file_path);
opts = setvartype(opts, t_col, "string");

T = readtable(file_path, opts);
T = sortrows(T, t_col);

t_0 = parse_time(T.(t_col)(1));
psi_0 = T.psi(1);

times = arrayfun(@(x) parse_time(x), T.(t_col)) - t_0;

% plot(T{:, ["x"]}, T{:, ["y"]});
xy = T{:, ["x", "y"]};
xy_ = (rot(psi_0) * xy')';
figure;
plot(times, xy(:, 1));
title("x");
figure;
plot(times, xy(:, 2));
title("y");
figure;
plot(times, T{:, ["v"]});
title("v");
figure;
plot(xy(:, 1), xy(:, 2));
title("x-y");
axis equal;

function res = parse_time(t)
    res = double(uint64(str2num(t)) / 1e6) / 1e3;
end
