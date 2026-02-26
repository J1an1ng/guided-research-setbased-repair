k = DefaultStateOrderInpJ;
dist = DisturbancesForInpJ1(k);
dt_of_constant_ref_input = 0.1;

%% Sampler
sampling_settings = SamplingSettings(k, dt_of_constant_ref_input);
bounds = sampling_settings.bounds;
interval_sampler = sampling_settings.create_interval_sampler(dist.state_err);
traj_sampler = sampling_settings.create_trajectory_sampler();

fprintf('# Initial intervals: %d \n', interval_sampler.num_samples);
fprintf('# Reference trajectories: %d \n', traj_sampler.num_samples);

%% Disturbances
% Input disturbance
W = 0.5 .* interval([-0.02; -0.3], [0.02; 0.3]);
% Measurement disturbance (from Anka's Master's thesis)
% V = interval([-0.01; -0.01; -0.02; -0.03; -0.115; -0.03], ...
%               [0.01;   0.01; 0.02;  0.03;  0.115;  0.03]);
%% FIXME uncertainties are too small for reality
V = 0.5 .* interval([-0.0004; -0.0004; -0.006; -0.002; -0.002; -0.006; -0.002], ...
                  [0.0004;  0.0004;  0.006;  0.002;  0.002;  0.006; 0.002]);

%% General settings
R_total = {};
% TODO: think about this again: does this need to equal the time that the
% controller is executed??
reach_set_dt = 0.01; % Hint: this time step size affects success of CORA!
v_threshold_for_stand_still = 1.0;

dyn = nonlinearSys(@continuously_controlled_model_with_inp_j, 16, 9);

%% Reachability Options
options.taylorTerms = 5;
options.zonotopeOrder = 20;
options.alg = 'lin';
options.tensorOrder = 3;
options.errorOrder = 25;
options.intermediateOrder = 25;
% options.intermediateTerms = 4;
% options.lagrangeRem.simplify = 'optimize';
% options.lagrangeRem.tensorParallel = true;

%% Start computation:
next_start_interval = interval_sampler.next();
c_start_interval = 0; % counter

primitive_counter = 0;
bulk_counter = 0;
bulk_size = 5;
num_failed = 0;

while ~isempty(next_start_interval)
    c_start_interval = c_start_interval + 1;

    % Keep track of accumulating size of R_total
    accumulate_counter = 0;

    fprintf('Start interval counter: %d \n', c_start_interval);
    display(next_start_interval);

    if interval_exceeds_friction_inp_j(k, next_start_interval, bounds)
        fprintf('Exceeds friction circle. Continue. \n');
        continue
    end

    lb = next_start_interval(1, :)';
    ub = next_start_interval(2, :)';
    i_next_start_interval = interval(lb, ub);
    c = center(i_next_start_interval);
    start_interval_with_ref_state = interval([lb; c; 0; 0], [ub; c; 0; 0]);

    params.U = cartProd(zonotope(W), zonotope(V));

    traj_sampler.reset();
    next_ref_traj = traj_sampler.next();
    c_ref_traj = 1;

    while ~isempty(next_ref_traj)
        fprintf('Ref traj counter: %d \n', c_ref_traj);
        fprintf('Ref traj: %s \n', next_ref_traj.toString());

        R0 = zonotope(start_interval_with_ref_state);
        options.timeStep = reach_set_dt;
        R_velocity = interval(project(R0, k.V));

        R_primitive = [];

        t = 0;
        ref_inputs = [];
        ref_states = [];

        successfull = true;

        % we simulate until quasi-standstill
        while R_velocity.sup > v_threshold_for_stand_still
            fprintf('t: %d \n', t);
            params.tStart = t;
            params.tFinal = t + dt_of_constant_ref_input;

            center_state = R0.center();
            current_ref_state = center_state(k.STATE_SIZE + 1:2 * k.STATE_SIZE);
            current_ref_state = current_ref_state';
            [j, dv] = next_ref_traj.inputsAt(t, current_ref_state);
            % add ref input to state
            Z = R0.Z;
            Z(2 * k.STATE_SIZE + 1:end, 1) = [dv; j];
            params.R0 = zonotope(Z);

            % TODO pre-allocating memory necessary?
            ref_inputs = [ref_inputs; [j dv]];
            ref_states = [ref_states; current_ref_state];

            try
                R_current = reach(dyn, params, options);
            catch
                fprintf('CORA reachset explosion');
                %% FIXME extract to own function
                %% Dump Current Parameters in txt file
                fid = fopen('Faulty_Primitives.txt', 'a+');
                [frows, fcols] = size(ref_inputs);
                fprintf(fid, '%c', '[');
                for frow = 1:frows
                    fprintf(fid, '  %d', ref_inputs(frows, :));
                    fprintf(fid, '%c \n', ']');
                end
                fprintf(fid, '\n \n', 1);
                fclose(fid);
                Input_Resulting_in_Error{c_start_interval, c_ref_traj} = ref_inputs;

                successfull = false;
                %% Break Current Primitive Simulation
                break
            end

            R_final = R_current(end).timePoint.set{end};
            R0 = R_final;

            R_primitive = add(R_primitive, R_current);

            t = t + dt_of_constant_ref_input;
            R_velocity = interval(project(params.R0, k.V));

            %% Check if ReachSet is within state bounds, but only if
            %% primitive is not finished yet
            % if R_velocity.sup > v_threshold_for_stand_still && ~Check_BoundaryCond(R_current, bounds, [3,7]) %Check only steering angle and acceleration
            %    successfull = false;
            %    fprintf('Reachset out of state boundaries\n');
            %    break
            % end
        end

        if successfull

            center_state = R0.center();
            last_ref_state = center_state(k.STATE_SIZE + 1:2 * k.STATE_SIZE);
            last_ref_state = last_ref_state';
            ref_states = [ref_states; last_ref_state];

            result.start_interval = i_next_start_interval;
            result.ref_traj_inputs = ref_inputs;
            result.ref_traj_input_dt = dt_of_constant_ref_input;
            result.ref_traj_states = ref_states;
            result.reachsets = R_primitive;
            R_total{c_start_interval, c_ref_traj} = result;

            primitive_counter = primitive_counter + 1;
        else
            num_failed = num_failed + 1;
        end

        % Containment check:
        %         Rtp = project(R0_ext,1:10);
        % TODO muss man die Endmengen nicht auch noch nach
        % Orientierung rotieren für den
        % Containment/Intervalgrößen-Check?
        %         R_endshift_center = R0.Z(:,1);
        %         R_end = RS_ext(end).timePoint.set{end,1};
        %         R_endshift_G = R_end.Z(:,2:end);
        %         R_endshift = zonotope(R_endshift_center, R_endshift_G);
        %         Contains = R0.contains(R_endshift);
        %         if Contains == 0
        %             fprintf('Warning: Zonotope Unchained')
        %         end

        next_ref_traj = traj_sampler.next();
        c_ref_traj = c_ref_traj + 1;

        if primitive_counter > 0 && mod(primitive_counter, bulk_size) == 0
            % we save the current bulk of primitives to mat-file
            bulk_counter = bulk_counter + 1;
            save_current_primitives_to_mat_file(bulk_counter, R_total);
            R_total = {};
            Input_Resulting_in_Error = {};
        end
    end

    next_start_interval = interval_sampler.next();
end

% we have to save the last bulk if it was not saved in the loop
if ~isempty(R_total)
    bulk_counter = bulk_counter + 1;
    save_current_primitives_to_mat_file(bulk_counter, R_total);
end

fprintf('# Successfull primitives: %d \n', primitive_counter);
fprintf('# Failed primitives: %d \n', num_failed);

%% Random Simulations (Scheitert, da params.paramInt Inputs möchte)
% simOpt.points = 40;
% params.paramInt = [0;0];
% params.tFinal = total_time(end);
% params.R0 = R0_ext;
% simRes = simulateRandom(Dyn, params, simOpt);

%% To create JSON: run `export_to_commonroad_json('out', 0.1, 'default', DefaultStateOrderInpJ)`
%% --> needs some time due to occupancy-computation.

function save_current_primitives_to_mat_file(bulk_counter, R_total)
    out_file_base = 'out/primitives';
    out_file = sprintf('%s_%d.%s', out_file_base, bulk_counter, 'mat');
    save(out_file, 'R_total');
end
