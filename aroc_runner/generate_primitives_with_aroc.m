%% Basic config
k = KSTTrailerStateOrderAroc;
dist = DisturbancesForInpA1(k);
primitive_duration = 0.5;
dt_constant_controller = 0.05; % 0.05; % TODO maybe change if required by reachsets
dt_controller = 0.01;

%% Sampler
sampling_settings = SamplingSettingsForShort(k, dt_controller);
bounds = sampling_settings.bounds;
interval_sampler = sampling_settings.create_interval_sampler(dist.state_err);
final_interval = interval(interval_sampler.final_interval(:, 1), interval_sampler.final_interval(:, 2));
traj_sampler = sampling_settings.create_trajectory_sampler();

display(final_interval);
fprintf('# Initial intervals: %d \n', interval_sampler.num_samples);
fprintf('# Reference trajectories: %d \n', traj_sampler.num_samples);

%% Post-processing
Post = @postprocessing_truck_adapted;

%% Set up AROC params
u_lb = zeros(k.INPUT_SIZE, 1);
u_ub = zeros(k.INPUT_SIZE, 1);
u_lb(k.INP_A) = bounds.a_bounds(1);
u_ub(k.INP_A) = bounds.a_bounds(2);
u_lb(k.INP_DELTA_V) = bounds.dv_bounds(1);
u_ub(k.INP_DELTA_V) = bounds.dv_bounds(2);

Params.U = interval(u_lb, u_ub);
Params.W = interval(-dist.input_dist, dist.input_dist);
Params.V = interval(-dist.state_err, dist.state_err);

% x_lb = zeros(2, 1);
% x_ub = zeros(2, 1);
% x_lb(1) = bounds.d_bounds(1);
% x_ub(1) = bounds.d_bounds(2);
% x_lb(2) = bounds.v_bounds(1);
% x_ub(2) = bounds.v_bounds(2);
% tmp = mptPolytope(interval(x_lb,x_ub));
% A = zeros(4,k.STATE_SIZE);
% A(:, [k.DELTA, k.V]) = tmp.P.A;
% Params.X = mptPolytope(A,tmp.P.b);

Params.tFinal = primitive_duration;

%% Set up AROC opts
Opts = [];

Opts.N = primitive_duration / dt_constant_controller;
Opts.Ninter = dt_constant_controller / dt_controller;
Opts.reachSteps = 1; % TODO subject to change
% Opts.extHorizon.active = 1;         % use extended optimization horizon
%% FIXME I think the horizon corresponds to N --> if N=1, the extHorizon
%% will probably have not effect
% Opts.extHorizon.horizon = 10;        % time steps for ext. horizon
% Opts.extHorizon.decay = 'fall';     % weight function for ext. horizon
%% TODO adapt these
% Opts.Q = diag([0,1,0,1,1,1]);      % state weighting matrix
q_values = 1 ./ interval_sampler.final_interval(:, 2);
q_values = q_values ./ mean(q_values);
q_values(k.PSI) = 10 * q_values(k.PSI);
q_values(k.Y) = 100 * q_values(k.Y);
Opts.Q = diag(q_values);
% Opts.R = 1e-6*eye(2);                  % input weighting matrix
% Opts.Q = diag([2,14,12,1,10,40]);      % state weighting matrix
% Opts.R = 1e-6*eye(2);
%% TODO maybe use extended horizon optimization

%% Reachability Options
Opts.cora.taylorTerms = 20;
Opts.cora.zonotopeOrder = 30;
Opts.cora.tensorOrder = 3;
% Opts.cora.lagrangeRem.simplify = 'optimize';
Opts.cora.errorOrder = 25;
Opts.cora.intermediateOrder = 20;

%% Start computation:
R_total = {};
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
    R0 = interval(lb, ub);
    x0 = center(R0);
    Params.R0 = R0;

    traj_sampler.reset();
    next_ref_traj = traj_sampler.next();
    c_ref_traj = 1;

    while ~isempty(next_ref_traj)
        fprintf('Ref traj counter: %d \n', c_ref_traj);
        fprintf('Ref traj: %s \n', next_ref_traj.toString());
        u = next_ref_traj.inputsAt(x0);

        %% CHANGE
        % get reference trajectory by simulating the system
        tspan = 0:dt_controller:primitive_duration;
        fun = @(t, x) aroc_truck(x, u, zeros(k.INPUT_SIZE, 1));

        % get reference trajectory by simulating the system
        [t, x] = ode45(fun, tspan, x0);

        %% FIXME is this good or bad??
        % provide reference trajectory as an additional input argument
        Opts.refTraj.x = x';
        Opts.refTraj.u = u' * ones(1, size(x, 1) - 1);
        % Opts.refTraj.Q = Opts.Q;

        Params.xf = x(end, :)';

        for i = 1:10
            fprintf('Try: %d \n', i);
            [obj, res] = generatorSpaceControl('truck', Params, Opts, Post);
            % [obj,res] = polynomialControl('truck',Params,Opts,Post);

            tiledlayout(3, 2);
            for y = 1:k.STATE_SIZE
                nexttile;
                plotOverTime(res.reachSet, y);
                title(sprintf('%d', i));
            end

            % check if final sets do not grow; we iterate over all dimensions
            % for the sake of better debugging possibility
            Rfin = shift_final_set_truck(obj.Rfin);
            success = true;
            display(final_interval);
            display(interval(Rfin));
            for j = 1:k.STATE_SIZE
                if ~contains(project(final_interval, j), project(Rfin, j))
                    success = false;
                    break
                end
            end

            if success
                break
            end

            % update weighting matrix
            scale = rad(interval(Rfin)) ./ rad(final_interval);
            scale = scale ./ scale(1);
            Opts.Q = diag(scale);

        end

        if ~success
            num_failed = num_failed + 1;
        end

        result.success = success;
        result.start_interval = R0;
        result.obj = obj;
        result.res = ref;

        R_total{c_start_interval, c_ref_traj} = result; %#ok<SAGROW>

        primitive_counter = primitive_counter + 1;

        next_ref_traj = traj_sampler.next();
        c_ref_traj = c_ref_traj + 1;

        if primitive_counter > 0 && mod(primitive_counter, bulk_size) == 0
            % we save the current bulk of primitives to mat-file
            bulk_counter = bulk_counter + 1;
            save_current_primitives_to_mat_file(bulk_counter, R_total);
            R_total = {};
        end

    end

    next_start_interval = interval_sampler.next();
end

function save_current_primitives_to_mat_file(bulk_counter, R_total)
    out_file_base = 'out/short_primitives';
    out_file = sprintf('%s_%d.%s', out_file_base, bulk_counter, 'mat');
    save(out_file, 'R_total');
end
