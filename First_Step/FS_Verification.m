duration = 0.01;

%% Define System
dyn = nonlinearSys(@vehicleDynamics_KST, 6, 2);

%% Specify measurement and input (model) errors
% Model state errors
x_err = 0.01; % [m]
y_err = 0.01; % [m]
delta_err = 0.0005; % [rad]
v_err = 0.06; % [m/s]
psi_err = 0.0005; % [rad]
ha_err = 0.0005; % [rad]
state_err = [x_err; y_err; delta_err; v_err; psi_err; ha_err];
state_err_interval = interval(-state_err, state_err);

% Model input errors
v_delta_err = 0.002; % [rad/s]
acc_err = 0.05; % [m/s**2]
input_err = [v_delta_err; acc_err];
input_err_interval = interval(-input_err, input_err);

%% Specify Initial Zonotope
% Specify initial state
initial_state = [0; 0; 0; 22; 0; 0];
initial_interval = interval(initial_state - state_err, initial_state + state_err);

%% Specify Control Input
u = [-0.55 -11];
U = interval(u' - input_err, u' + input_err);

%% Specify reachability params and options
% params
params.tStart = 0;
params.tFinal = params.tStart + duration;
params.R0 = zonotope(initial_interval);
params.U = zonotope(U);
% options
options.alg = 'lin';
options.timeStep = 1;
options.tensorOrder = 2;
options.taylorTerms = 20;
options.zonotopeOrder = 30;
options.errorOrder = 25;
options.intermediateOrder = 20;
try
    tic;
    R_current = reach(dyn, params, options);
    toc;
end

%% Evaluation

%% Specify (supposed) containing zonotope:
x_des = 0.15;
y_des = 0.05;
delta_des = 0.012;
v_des = 1;
psi_des = 0.008;
ha_des = 0.016;
desired = [x_des; y_des; delta_des; v_des; psi_des; ha_des];
desired_interval = interval(-desired, desired);
containing_zonotope = zonotope(desired_interval);

%% Get zonotope of last step
inner_zonotope = R_current.timeInterval.set{end};
inner_interval = interval(inner_zonotope);

%% Transform last step zonotope to containing zonotope:
inner_center = inner_zonotope.center;
shifted_inner_zonotope = zonotope(inner_zonotope - inner_center);
% Rotate the set
phi = inner_center(5, 1);
T = RotMat(phi, 10, [1, 2]);
shifted_inner_zonotope = t * shifted_inner_zonotope;
display(containing_zonotope.contains(shifted_inner_zonotope));

tiledlayout(3, 2);
for kk = 1:6
    nexttile;
    plot(shifted_inner_zonotope, [1 2], 'FaceColor', 'b', 'FaceAlpha', 0.5);
    hold on;
    plot(containing_zonotope, [1 2], 'FaceColor', 'y', 'FaceAlpha', 0.5);
    title({kk});
end

display(desired_interval);
display(interval(shifted_inner_zonotope));

figure;
plotOverTime(R_current, 2);

function RotMat = RotMat(angle, dimension, axes)
    % angle: To rotate this amount of degrees
    % dimension: Dimensions of the object to be rotated
    % axes: axes defining the plane in which to rotate (axes(1) and axes(2)).
    RotMat = eye(10);
    RotMat(axes(1), axes(1)) = cos(angle);
    RotMat(axes(1), axes(2)) = -sin(angle);
    RotMat(axes(2), axes(1)) = sin(angle);
    RotMat(axes(2), axes(2)) = cos(angle);
end
