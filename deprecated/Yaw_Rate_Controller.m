function u = Yaw_Rate_Controller(state, ref_state)
    k = DefaultStateOrderInpA;
    l_wb = VehicleParams.l_wb;
    trailer_l_wb = VehicleParams.trailer_l_wb;
    L = VehicleParams.a + VehicleParams.b;

    %% Parse State
    % actual initial state (controlled)
    x = state(k.X);
    y = state(k.Y);
    delta = state(k.DELTA);
    v = state(k.V);
    psi = state(k.PSI);
    dpsi = v * tan(delta) / l_wb;

    % reference state
    x_ref = ref_state(k.X);
    y_ref = ref_state(k.Y);
    delta_ref = ref_state(k.DELTA);
    v_ref = ref_state(k.V);
    psi_ref = ref_state(k.PSI);
    dpsi_ref = v_ref * tan(delta_ref) / l_wb;

    %% Compute Control Inputs
    %% Yaw rate Control & PID Control
    d_v_scale = 1;
    a_scale = 1.5;
    w(4) = 25; % 80;  % weight for longitudinal error 16
    w(5) = 30; % 100;  % weight for velocity error
    w(6) = 0;  % weight for acceleration error

    epsilon_lat = -sin(psi_ref) * (x_ref - x) + cos(psi_ref) * (y_ref - y);
    epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);
    Delta_delta = epsilon_lat / (v + 1.0);

    ddpsi = 40 * (50 * (psi_ref - psi) + (dpsi_ref - dpsi) + 1.5 * Delta_delta) + 0.5 * (50 * (dpsi_ref - dpsi));
    u(k.INP_DELTA_V, 1) = d_v_scale * (cos(delta))^2 * (L * ddpsi) / (v + 1);
    u(k.INP_A, 1) = a_scale * (w(4) * epsilon_long + w(5) * (v_ref - v) + w(6));

    %% Stanley Control & PID Control
    % d_v_scale = 1;
    % w(1) = 10; % weight for orientation error
    % %stanley control
    % k_lat = 20; % weight for lat. error in stanley control
    % k_soft = 0.1; % k_soft is to void demoninator v = 0
    % k_yaw = 2; % the factor for error of yaw rate, for stability and void oscillation
    % alphaf = vy/v - delta; % the slip angle of front axle tire
    % delta_f = (Psir-Psi) + atan(k_lat*epsilon_lat/(v+k_soft)) + k_yaw*(dPsir-dPsi) - alphaf;
    % j_scale = 1;
    % w(2) = 80;  % weight for longitudinal error
    % w(3) = 100;  % weight for velocity error
    % w(4) = 20;  % weight for acceleration error
    %
    % u(k.DELTA_V, 1) = d_v_scale * (ref_input(1) + w(1)*(delta_f - delta)); % ref_input(1) is the dot(delta)_r
    % u(k.J, 1) = j_scale * (w(2)*epsilon_long + w(3)*(vr-v)+w(4)*(accr-acc));

    %% Add Input disturbances
    % u = u + disturb_input;
    %% remove control with velocity too small
    % u(k.DELTA_V, 1) = u(k.DELTA_V, 1)*(1-input_stop) + ref_input(1)*input_stop;
    % u(k.J, 1) = u(k.J, 1)*(1-input_stop) + ref_input(2)*input_stop;
    % u = u*(1-input_stop);

end
