function inp = create_inp_for_a_delta(~, i, ~, a, desired_delta, ~)
    inp(i.DELTA, 1) = desired_delta;
    inp(i.A, 1) = a;
end
