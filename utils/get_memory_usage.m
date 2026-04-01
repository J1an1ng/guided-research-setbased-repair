function [total_memory, used_memory] = get_memory_usage()
    % Get total memory usage from the system
    [~, result] = system('free -m');
    lines = strsplit(result, '\n');
    mem_line = strsplit(lines{2});
    total_memory = str2double(mem_line{2});
    used_memory = str2double(mem_line{3});
end
