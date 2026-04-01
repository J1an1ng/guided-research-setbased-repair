%% FIXME this has to be adapted to use the Generator-framework;
%% it needs an own state processor

k = DefaultStateOrderInpA;
dist = DisturbancesForInpA1(k);
primitive_duration = 0.5; % 0.75; %1.0;
dt_controller = 0.01;
primitive_num_steps = primitive_duration / dt_controller;
pure_pursuit_lookahead_time = 0.1;
pure_pursuit_lookahead_num_steps = pure_pursuit_lookahead_time / dt_controller;

%% Sampler
sampling_settings = SamplingSettingsForShort(k, dt_controller);
bounds = sampling_settings.bounds;
interval_sampler = sampling_settings.create_interval_sampler(dist.state_err);
final_interval = interval(interval_sampler.final_interval(:, 1), interval_sampler.final_interval(:, 2));
traj_sampler = sampling_settings.create_trajectory_sampler();

fprintf('# Initial intervals: %d \n', interval_sampler.num_samples);
fprintf('# Reference trajectories: %d \n', traj_sampler.num_samples);
display(final_interval);

%% Disturbances
% Input disturbance
W = interval(-dist.input_dist, dist.input_dist);
% Measurement disturbance
state_err_interval = [-dist.state_err dist.state_err];
state_err_interval_ext = [zeros(k.STATE_SIZE, 2); state_err_interval];
V = interval(state_err_interval_ext(:, 1), state_err_interval_ext(:, 2));
display(V);

R_total = {};

dyn = nonlinearSys(@discretely_pure_pursuit_controlled_model, 3 * k.STATE_SIZE, 2);

%% Reach options
options.taylorTerms = 20;
options.zonotopeOrder = 30;
options.alg = 'lin';
options.tensorOrder = 3;
options.errorOrder = 25;
options.intermediateOrder = 20;

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

    if interval_exceeds_friction_inp_a(k, next_start_interval, bounds)
        fprintf('Exceeds friction circle. Continue. \n');
        continue
    end

    lb = next_start_interval(1, :)';
    ub = next_start_interval(2, :)';
    i_next_start_interval = interval(lb, ub);
    current_ref_state = center(i_next_start_interval);
    R0 = zonotope(i_next_start_interval);

    params.U = zonotope(W);

    traj_sampler.reset();
    next_ref_traj = traj_sampler.next();
    c_ref_traj = 1;

    while ~isempty(next_ref_traj)
        fprintf('Ref traj counter: %d \n', c_ref_traj);
        fprintf('Ref traj: %s \n', next_ref_traj.toString());

        options.timeStep = dt_controller;

        R_primitive = [];

        t = 0;
        ref_inputs = [];
        ref_states = current_ref_state';

        successfull = true;

        allSimRes = {};

        %% Create ref traj
        for i = 1:primitive_num_steps
            u = next_ref_traj.inputsAt(current_ref_state);

            tspan = [0 dt_controller];
            fun = @(t, x) model_with_inp_a(x, u);
            % simulate ref state
            [t_res, x] = ode45(fun, tspan, current_ref_state);
            current_ref_state = x(end, :)';

            ref_inputs = [ref_inputs; u]; %#ok<AGROW>
            ref_states = [ref_states; current_ref_state']; %#ok<AGROW>
        end

        %% FIXME is this a copy??
        %% We continue simulating the ref trajectory for zero input
        u = zeros(1, 2);
        ref_states_extended = ref_states;
        for i = 1:pure_pursuit_lookahead_num_steps
            current_ref_state(k.DELTA) = 0; % FIXME is this good or not??

            tspan = [0 dt_controller];
            fun = @(t, x) model_with_inp_a(x, u);
            % simulate ref state
            [t_res, x] = ode45(fun, tspan, current_ref_state);
            current_ref_state = x(end, :)';

            ref_states_extended = [ref_states_extended; current_ref_state']; %#ok<AGROW>
        end

        for i = 1:primitive_num_steps
            fprintf('t: %d \n', t);
            params.tStart = t;
            params.tFinal = t + dt_controller;

            ref_state_for_long = ref_states(i, :)';
            ref_state_for_pp = ref_states_extended(i + pure_pursuit_lookahead_num_steps, :)';

            %% IMPORTANT: this order has to match the one in the controlled model!!
            ref_state_for_contr = zeros(6, 1);
            ref_state_for_contr(1) = ref_state_for_long(k.X);
            ref_state_for_contr(2) = ref_state_for_long(k.Y);
            ref_state_for_contr(3) = ref_state_for_long(k.V);
            ref_state_for_contr(4) = ref_state_for_long(k.PSI);
            ref_state_for_contr(5) = ref_state_for_pp(k.X);
            ref_state_for_contr(6) = ref_state_for_pp(k.Y);

            R0_center = center(R0);
            R0_generators = generators(R0);
            % We duplicate each state in the set...
            R0 = zonotope([R0_center; R0_center], [R0_generators; R0_generators]);
            % ... and add the disturbance to the state copy:
            R0 = R0 + V;
            % Then, we add the reference state:
            R0_center = center(R0);
            R0_generators = generators(R0);
            R0 = zonotope([R0_center; ref_state_for_contr], ...
                [R0_generators; zeros(6, size(R0_generators, 2))]);
            params.R0 = R0;

            if i == 1
                % random simulation
                simOptions.points = 20;
                simOptions.fracVert = 0.5;
                simOptions.fracInpVert = 1;
                simOptions.nrConstInp = 1;
                % random simulation
                allSimRes{1} = simulateRandom(dyn, params, simOptions);
            else
                simParams.tStart = params.tStart;
                simParams.tFinal = params.tFinal;
                lastSimRes = allSimRes{i - 1};
                newSimRes = [];
                for l = 1:length(lastSimRes.x)
                    x0 = lastSimRes.x{l};
                    x0_new = x0(end, 1:k.STATE_SIZE)';
                    x0_new = [x0_new; x0_new] + randPoint(V);
                    x0_new = [x0_new; ref_state_for_contr]; %#ok<AGROW>
                    simParams.x0 = x0_new;
                    simParams.u = randPoint(params.U);
                    [t_res, x_res] = simulate(dyn, simParams);
                    newSimRes = add(newSimRes, simResult({x_res}, {t_res}));
                end
                allSimRes{i} = newSimRes; %#ok<SAGROW>
            end

            R_current = reach(dyn, params, options);

            R_final = R_current(end).timePoint.set{end};
            %% project to actual state (i.e. remove initial state and ref state)
            R0 = project(R_final, 1:k.STATE_SIZE);

            R_primitive = add(R_primitive, R_current);

            t = t + dt_controller;
        end

        simRes = [];
        for i = 1:length(allSimRes)
            simRes = add(simRes, allSimRes{i});
        end

        tiledlayout(3, 2);
        for i = 1:k.STATE_SIZE
            nexttile;
            plotOverTime(R_primitive, i);
            hold on;
            plotOverTime(simRes, i);

            title(sprintf('%d', i));
        end

        success = true;
        R0 = shiftFinalSet_truck_default(R0);
        display(final_interval);
        display(interval(R0));
        for j = 1:k.STATE_SIZE
            if ~contains(project(final_interval, j), project(R0, j))
                success = false;
                % break
            end
        end

        if successfull
            result.start_interval = i_next_start_interval;
            result.ref_traj_inputs = ref_inputs;
            result.ref_traj_input_dt = dt_controller;
            result.ref_traj_states = ref_states;
            result.reachsets = R_primitive;
            R_total{c_start_interval, c_ref_traj} = result; %#ok<SAGROW>

            primitive_counter = primitive_counter + 1;
        else
            num_failed = num_failed + 1; %#ok<UNRCH>
        end

        next_ref_traj = traj_sampler.next();
        c_ref_traj = c_ref_traj + 1;

    end

end
