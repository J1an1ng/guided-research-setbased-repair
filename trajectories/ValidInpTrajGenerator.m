classdef ValidInpTrajGenerator < handle
    properties (Access = private)
        inp_traj_gen AbstractInpTrajGenerator
        simulator Simulator
        duration double
        dt_step double
    end

    methods

        function obj = ValidInpTrajGenerator(inp_traj_gen, terminal_cond, ...
                duration, dt_step, time_decimal_places, model, ...
                state_order, inp_order, params, constraints, ...
                extended_horizon, process_final_state)
            obj.inp_traj_gen = inp_traj_gen;

            if ~exist("extended_horizon", "var")
                extended_horizon = 0;
                process_final_state = 0;
            end
            obj.simulator = Simulator(terminal_cond, ...
                time_decimal_places, model, state_order, inp_order, params, constraints, ...
                extended_horizon, process_final_state);
            obj.duration = duration;
            obj.dt_step = dt_step;
        end

        function [trajectories, discarded_trajs] = generate(obj, initial_state)
            all_trajectories = obj.inp_traj_gen.generate(initial_state);
            is_valid = cellfun(@(traj) obj.simulator.simulate(traj, initial_state, obj.duration, obj.dt_step), ...
                all_trajectories);
            %% Hint: we do generally not "repair" a trajectory,
            %% if the constraints (i.e. state/input bounds, friction) are
            %% violated, but we discard the trajectory in this case.
            %% If we were repairing trajectories, it would be not
            %% determinable anaymore which trajectories we sample,
            %% e.g., we would possibly sample very similar trajectories
            %% multiple time.
            %% Instead, we assume that the underlying trajectory generator
            %% :inp_traj_gen is already taking care, that - if a trajectory
            %% violates the constraints - we will not miss a crucial maneuver.
            trajectories = all_trajectories(is_valid);
            discarded_trajs = all_trajectories(~is_valid);
        end

    end
end
