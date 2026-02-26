function [data] = read_file(folder, id, time_idcs)
%% Read a measurement file from folder with {id}
% Return most important data as struct

    % load measurement
    name = sprintf('Data_10ms_%03d', id);
    path = strcat(folder, '/', name); 
    try
        load(path)
    catch
        search_name = sprintf('Data_10ms_*%03d.mat', id);
        S = dir(fullfile(folder, search_name));
        name = S.name(1:end-4); 
        path = strcat(folder, '/', name); 
        load(path); 
    end
    
    % get all signal names
    names = eval(sprintf('{%s.Y.Name}', name));
    
    % MAKROS search string in measurements
    id_from_name = @(field) find(ismember(names, field));
    cmd = @(field) sprintf('%s.Y(%d).Data', name, id_from_name(field));
    x_time_series = eval(sprintf('%s.X', name));
    
    x   = eval(cmd('Data_10ms.ADMA_CAN_inputs.UTM_Position_POI1_absolute.f64X'));       % x (m)
    y   = eval(cmd('Data_10ms.ADMA_CAN_inputs.UTM_Position_POI1_absolute.f64Y'));       % y (m)
    
    psi = eval(cmd('Data_10ms.ADMA_CAN_inputs.INS_Angle_and_GPS_COG.INS_Yaw'));         % yaw (rad) 
    vX  = eval(cmd('Data_10ms.ADMA_CAN_inputs.UTM_Position_POI1_absolute.f64Vx'));      % Vx (m/s)                       <--- more noisy than vf
    vf = eval(cmd('Data_10ms.C_CAN_Input.EBC2_VAR1.frontAxleSpeed')) / 3.6;             % front axle speed (m/s)
    swa_req = eval(cmd('Main_Bus_out.Request_Bus.RequestsLat.sWA_req_rad'));            % SWA req (rad)
    swa_curr = eval(cmd('Data_10ms.Servotwin_Input.EASI2_BASE.CurrentSteeringAngle'));  % current SWA (rad)     
    swa_act = eval(cmd('Main_Bus_out.writeToActuatorsBus.VehCtrl_PHYS_Output.ADSCprop2_PHYS.TrajCtrl_reqSWA'));
    auto = cast(eval(cmd('Data_10ms.VehCtrl_Inputs.ADSSprop2.VehCtrl_operationMode')), 'double');       % automation mode


    %% Reference trajectory - the 3rd element is the one of the current time
    x_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_LLF.ReferenceStates.x_m[2]'));
    %x_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_V1.ReferenceStates.x_m[2]'));
    y_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_LLF.ReferenceStates.y_m[2]'));
    %y_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_V1.ReferenceStates.y_m[2]'));
    v_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_LLF.ReferenceStates.vX_ms[2]'));
    psi_ref = eval(cmd('Main_Bus_out.Reference_Bus.PlanerMessage_LLF.ReferenceStates.yaw_rad[2]'));

    from = time_idcs(1);
    to = time_idcs(2);

    data.output.x = x(from:to);
    data.output.y = y(from:to);
    data.output.v = vf(from:to);
    data.output.psi = psi(from:to);

    data.ref.x = x_ref(from:to);
    data.ref.y = y_ref(from:to);
    data.ref.v = v_ref(from:to);
    data.ref.psi = psi_ref(from:to);

    return

    
    %swa2 = eval(cmd('Data_10ms.C_figureCAN_Input.VDC2_BASIS.SteeringWheelAngle'));
    
    % POI lateral velocities make no sense!
    vx1 = eval(cmd('Data_10ms.ADMA_CAN_inputs.INS_Velocity_horizontal.INS_Vel_Hor_X'));
    vy1 = eval(cmd('Data_10ms.ADMA_CAN_inputs.INS_Velocity_horizontal.INS_Vel_Hor_Y'));
    
    vx2 = eval(cmd('Data_10ms.ADMA_trailer_inputs.INS_Velocity_horizontal.INS_Vel_Hor_X'));
    vy2 = eval(cmd('Data_10ms.ADMA_trailer_inputs.INS_Velocity_horizontal.INS_Vel_Hor_Y'));
    
    psi2 = eval(cmd('Data_10ms.ADMA_trailer_inputs.INS_Angle_and_GPS_COG.INS_Yaw'));
    x2 = eval(cmd('Data_10ms.ADMA_trailer_inputs.UTM_Position_POI1_absolute.f64X')); 
    y2 = eval(cmd('Data_10ms.ADMA_trailer_inputs.UTM_Position_POI1_absolute.f64Y'));
        
    % SWA is saturated by the CTRL model
    swa_sat = swa_req;  % for now just a copy of the signal
    
    psi_dot = eval(cmd('Data_10ms.C_CAN_Input.VDC2_BASIS.YawRate'));
    
    psi_dot1 = eval(cmd('Data_10ms.ADMA_CAN_inputs.Rates_horizontal.Rate_Hor_Z'));
    psi_dot2 =  eval(cmd('Data_10ms.ADMA_trailer_inputs.Rates_horizontal.Rate_Hor_Z'));

    ay1 =eval(cmd('Data_10ms.C_CAN_Input.VDC2_BASIS.LateralAcceleration'));
    
    % control errors
    ex  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_x'));
    ey  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_y'));
    e_psi  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_yaw'));
    es  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_s'));
    et  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_time'));
    ev  = eval(cmd('Main_Bus_out.SignalProcess_Bus.controlErrors.error_v'));
    
    aX_req = eval(cmd('Main_Bus_out.Request_Bus.RequestsLong.aX_req_ms2')); 
    gear_shift = double(eval(cmd('Data_10ms.C_CAN_Input.ETC1_MAN.transmissionShiftInProcess'))); 
    
    gamma = eval(cmd('Data_10ms.HitchSenIn.Winkel1')) + eval(cmd('Data_10ms.HitchSenIn.Winkel2')); 
    gamma = 0.5*gamma *pi/180;
    
    %ax2 = eval(cmd('Main_Bus_out.Reference_Bus.egoData_Ctrl_BusObject.aY_ms2'));
    
    % Read brake values   
    brake = struct();
    brake.psi_dot = eval(cmd('Data_10ms.C_CAN_Input.VDC2_BASIS.YawRate'));
    brake.ay = eval(cmd('Data_10ms.C_CAN_Input.VDC2_BASIS.LateralAcceleration'));
    brake.ax = eval(cmd('Data_10ms.C_CAN_Input.VDC2_BASIS.LongitudinalAcceleration'));
    
    outshaft = eval(cmd('Data_10ms.Servotwin_Input.AESS1_BASE.ActualOutputShaftAngle'));
    outshaft_req = eval(cmd('Data_10ms.Servotwin_Input.AESS1_BASE.RequestedOutputShaftAngle'));


    %% Write data struct
    data = struct();
    data.x = x; 
    data.y = y; 
    data.psi = psi; 
    data.psi_dot = psi_dot;
    data.psi_dot1 = psi_dot1 * pi/180;
    data.vf = vf; % front axle speed

    data.vX = vX;  % <- noisy for bad ADMA position
    % Projection at POI induces noise!
    data.vx1 = vx1;  
    data.vy1 = vy1;  
    data.ay1 = ay1; 

    % Trailer
    % only add trailer data if available
    if any(vx2 > 0)
        data.vx2 = vx2;  
        data.vy2 = vy2;
        data.psi_dot2 = psi_dot2 * pi/180;    
        data.x2 = x2; 
        data.y2 = y2; 
        data.psi2 = psi2; 
        data.gamma = gamma; 
    end

    %% Ref state
    data.x_ref = x_ref;
    data.y_ref = y_ref;
    data.v_ref = v_ref;
    data.psi_ref = psi_ref;

    % CONTROL signals
    data.swa_req = swa_req;     % from Control 
    data.swa_curr = swa_curr;   % Actuator SWA
    data.a_req = aX_req; 
    data.swa_act = swa_act;     % Limited SWA
    
    data.outshaft = outshaft;   % OutputShaft Angle EAS2
    data.outshaft_req = outshaft_req; 

    N = length(x); 
    dt = 1e-2; 
    data.t = (1:N)*dt - dt; 

    % Errors
    data.error.x = ex; 
    data.error.y = ey; 
    data.error.v = ev; 
    data.error.psi = e_psi;
    data.error.t = et; 
    data.error.s = es; 

    % gear shifts
    data.gear_shift = gear_shift;
    
    % automatization
    data.auto = auto; 
    
    % brake values
    data.brake = brake;

    data.swa_dot = gradient(data.swa_curr, 1e-2);
end

