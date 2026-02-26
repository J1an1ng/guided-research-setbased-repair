function res = max_abs_in_interval(interv, idx)
    inf_sup = [interv.inf(idx) interv.sup(idx)];
    res = max(abs(inf_sup));
end
