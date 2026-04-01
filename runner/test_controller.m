%% Little script for testing a controlled system by simulating trajectories

%% Model settings
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;

%% Configs
params = create_kst_trailer_params_1();

state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);

time_end = 0.5;
time_step = 0.01;
n = time_end / time_step;
tspan = 0:time_step:time_end;

% input + state disturbances: partially from Anka's thesis, partially from
% Internet, partially "guessed"
max_abs_dist = [0.08; 0.75; 0.02; 0.02; 0.05; 0.06; 0.004; 0.05];
dist_traj = create_disturbance_traj(max_abs_dist, n);
% display(dist_traj);
get_dist = @(t) dist_traj(:, min(fix(t / time_step) + 1, n));

% initial state:
x0 = [0.1;  0.1;  0.1;  20;  0.04; 0  % actual state
      0;    0;    0;    21;  0;    0  % ref state
      -0.17; -1.5]; % ref input

fun = @(t, x) controlled_model_with_inp_a(x, get_dist(t));

% get reference trajectory by simulating the system
[t, x] = ode45(fun, tspan, x0);

% display(x);

actual_state = x(:, 1:2);
ref_state = x(:, 7:8);

plot(ref_state(:, 1), actual_state(:, 2), 'blue');
axis equal;
figure;
plot(actual_state(:, 1), actual_state(:, 2), 'red');
axis equal;

function dist_traj = create_disturbance_traj(max_values, n)
    state_size = size(max_values);
    assert(state_size(2) == 1);
    state_size = state_size(1);

    value_ranges = 2 * max_values;

    dist_traj = repmat(value_ranges, [1 n]) .* rand(state_size, n) + repmat(-max_values, [1 n]);
end
