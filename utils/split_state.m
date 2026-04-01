function [s_1, s_2] = split_state(s, n)
    s_1 = s(1:n);
    s_2 = s(n + 1:end);
end
