function s = clean_interval_json_string(s)
    s = strrep(s, ':', '');
    s = strrep(s, '"', '');
    s = s(2:end - 1);
end
