function fun = create_acc_function(acc_profile, duration)
    dt_per_acc = duration / length(acc_profile);

    function a = get_acc(t)
        idx = min(floor(t / dt_per_acc) + 1, length(acc_profile));
        a = acc_profile(idx);
    end

    fun = @get_acc;
end
