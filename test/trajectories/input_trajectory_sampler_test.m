% FIXME adapt to new parameters
sampler = InputTrajectorySampler(0.1, 0.1, 0.2, 0.2, [-10. -20 -30; -20 -40 -60], 0.5);

inp = sampler.next();
c = 0;
while ~isempty(inp)
    display(c);

    for t = 0:0.1:2
        [jerk, steering_angle_speed] = inp.inputsAt(t);
        fprintf('%d || %d\n', jerk, steering_angle_speed);
    end

    inp = sampler.next();
    c = c + 1;
end
