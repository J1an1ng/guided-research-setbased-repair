clc;
clear;
N_Sim = 100;
parameter = parameters_vehicle4;
total_time = linspace(0, 1, N_Sim);
delta_t = total_time(2) - total_time(1);

%% Define Reference Input
jerk = 0.0;
v_delta = 0.0;
Input_jerk = linspace(jerk, jerk, N_Sim);
Input_v_delta = linspace(v_delta, v_delta, N_Sim);
%% Define Initial Conditions for Reference
Ref_x0 = 0;
Ref_y0 = 0.5;
Ref_delta0 = 0;
Ref_v0 = 22;
Ref_Psi0 = 0;
Ref_a0 = 0;
Ref_initial_state = [Ref_x0; Ref_y0; Ref_delta0; Ref_v0; Ref_Psi0; Ref_a0];
%% Define Initial Conditions for Tracking
Track_x0 = 0;
Track_y0 = 0;
Track_delta0 = 0;
Track_v0 = 30;
Track_Psi0 = 0;
Track_a0 = 0;
Track_initial_state = [Track_x0; Track_y0; Track_delta0; Track_v0; Track_Psi0; Track_a0];
%% Initial State
Initial_State(1:length(Ref_initial_state)) = Track_initial_state;
Initial_State(length(Track_initial_state) + 1:length(Track_initial_state) + length(Ref_initial_state)) = Ref_initial_state;
Initial_State = Initial_State';

X = [Initial_State, zeros(length(Initial_State), N_Sim - 1)];

for k = 1:N_Sim - 1
    display(k);
    [~, x_temp] = ode45(@(t, x)KS_Controlled_Synth_J(x, [0; 0; 0; 0; 0; 0; 0; 0], [Input_v_delta(k), Input_jerk(k)]), [0, delta_t], X(:, k)');
    X(:, k + 1) = x_temp(end, :)';
end

lenni = length(Ref_initial_state);
figure;
plot(X(1, :), X(2, :));
hold on;
plot(X(lenni + 1, :), X(lenni + 2, :));
figure;
plot(total_time, X(3, :));
hold on;
plot(total_time, X(lenni + 3, :));
figure;
plot(total_time, X(4, :));
hold on;
plot(total_time, X(lenni + 4, :));
figure;
plot(total_time, X(5, :));
hold on;
plot(total_time, X(lenni + 5, :));
figure;
plot(total_time, X(6, :));
hold on;
plot(total_time, X(lenni + 6, :));
