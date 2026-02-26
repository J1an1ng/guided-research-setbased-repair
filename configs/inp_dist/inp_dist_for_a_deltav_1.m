function inp_dist = inp_dist_for_a_deltav_1(inp_order)
    arguments
        inp_order InpOrderADeltaV
    end

    inp_dist = zeros(inp_order.SIZE, 1);
    % From here: https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/
    % images/responsive/supporting/events/matlab-expo-2021/proceedings/
    % matlab-expo-2021-develop-a-by-wire-braking-system-for-l4-autonomy-trucks-edt.pdf
    inp_dist(inp_order.A) = 0.1;

    inp_dist(inp_order.DELTA_V) = 0.002;
end
