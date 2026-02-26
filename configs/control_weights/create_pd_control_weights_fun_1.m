function [weights_size, f] = create_pd_control_weights_fun_1(k)
    weights_size = 5;
    f = @pd_control_weights_fun_1;

    function w = pd_control_weights_fun_1(p, ~)
        v_border = 10;
        if p.start_set.enclosing_itv.inf(k.V) > v_border
            w = [34.4; 1187.5; 23.4; 46.9; 162.5];
            %% Test for lateral control
            % w = [100; 1187.5; 23.4; 46.9; 162.5];
        else
            w = [7.0312; 421.8750; 47.2656; 175.7812; 128.1250];
        end
    end

end
