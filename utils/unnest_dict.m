function d = unnest_dict(d)
    % Replaces transitive chains in the dictionary by the direct
    % connections; the resulting dictionary does not contain the
    % intermediate entries anymore.
    d_copy = dictionary(d);
    used_keys = dictionary(uint32.empty, logical.empty);
    while d_copy.numEntries > 0
        d_keys = d_copy.keys();
        k = d_keys(1);
        d_copy = remove(d_copy, k);
        v = d(k);

        while isKey(d, v)
            new_v = d(v);
            d(k) = new_v;
            used_keys = used_keys.insert(v, true);
            v = new_v;
        end
    end
    d = d.remove(used_keys.keys());
end
