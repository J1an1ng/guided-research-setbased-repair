myfolder = '~/CommonRoad/commonroad-highway-fail-safe-planning/Test_Primitives';
filePattern = fullfile(myfolder, '*.mat');
theFiles = dir(filePattern);

ks_bounds = SamplingSettings(0.1).ks_bounds;
ks_bounds.a_bounds(2) = 2.0;

S = {};

%%
% fig2 = figure();
% ax1 = subplot(4,1,1, 'Parent', fig2); title('All', 'Interpreter', 'latex'); %all trajectories
% ax2 = subplot(4,1,2, 'Parent', fig2); title('Inside Steering Bounds', 'Interpreter', 'latex'); % inside delta bounds
% ax3 = subplot(4,1,3, 'Parent', fig2); title('Inside Acceleration Bounds', 'Interpreter', 'latex'); % inside acceleration bounds
% ax5 = subplot(4,1,4, 'Parent', fig2); title('Inside Bounds', 'Interpreter', 'latex'); % inside both bounds

% n_total = 0;
% n_delta = 0;
% n_acc = 0;

% for kk=1:size(theFiles,1)
%    display(kk)
%    load(theFiles(kk).name);
% [S(kk).fail_traj, S(kk).fail_where] = ReachSet_CheckBound(R_total);
%    non_empty_cells = find(~cellfun(@isempty, R_total));
%    for jj = non_empty_cells'
%        axes(ax1); hold on; plot(R_total{jj}.ref_traj_states(:,1), R_total{jj}.ref_traj_states(:,2));
%        axes(ax1); hold on; plot(R_total{jj}.ref_traj_states(:,1), -R_total{jj}.ref_traj_states(:,2));
%        cumulative_delta = true;
%        cumulative_acc = true; cumulative_total = true;
%        for ll = 1:length(R_total{jj}.reachsets)
%        cumulative_delta = cumulative_delta & Check_BoundaryCond(R_total{jj}.reachsets(ll), ks_bounds, [3]);
%        cumulative_acc = cumulative_acc & Check_BoundaryCond(R_total{jj}.reachsets(ll), ks_bounds, [7]);
%        cumulative_total = cumulative_total & Check_BoundaryCond(R_total{jj}.reachsets(ll), ks_bounds, [3,7]);
%        end
%        if cumulative_delta
%            axes(ax2); hold on; plot(R_total{jj}.ref_traj_states(:,1), R_total{jj}.ref_traj_states(:,2));
%            hold on; plot(R_total{jj}.ref_traj_states(:,1), -R_total{jj}.ref_traj_states(:,2));
%            n_delta = n_delta+1
%        end
%        if cumulative_acc
%            axes(ax3); hold on; plot(R_total{jj}.ref_traj_states(:,1), R_total{jj}.ref_traj_states(:,2));
%            hold on; plot(R_total{jj}.ref_traj_states(:,1), -R_total{jj}.ref_traj_states(:,2));
%            n_acc = n_acc+1
%        end
%        if cumulative_total
%            axes(ax5); hold on; plot(R_total{jj}.ref_traj_states(:,1), R_total{jj}.ref_traj_states(:,2));
%            hold on; plot(R_total{jj}.ref_traj_states(:,1), -R_total{jj}.ref_traj_states(:,2));
%            n_total = n_total+1
%        end
%    end
% end

%%
fig = figure();
ax1 = subplot(2, 4, 1, 'Parent', fig);
title('Y-Coord [$m$]', 'Interpreter', 'latex');
xlabel('X-Coord [$m$]', 'Interpreter', 'latex');
ax2 = subplot(2, 4, 2, 'Parent', fig);
title('$\delta$ [rad]', 'Interpreter', 'latex');
ax3 = subplot(2, 4, 3, 'Parent', fig);
title('$\Psi$ [rad]', 'Interpreter', 'latex');
ax4 = subplot(2, 4, 4, 'Parent', fig);
title('Acc $[\frac{m}{s^2}]$', 'Interpreter', 'latex');
ax5 = subplot(2, 4, 5, 'Parent', fig);
xlabel('X-Coord [$m$]', 'Interpreter', 'latex');
ax6 = subplot(2, 4, 6, 'Parent', fig);
xlabel('Time [$s$]', 'Interpreter', 'latex');
ax7 = subplot(2, 4, 7, 'Parent', fig);
ax8 = subplot(2, 4, 8, 'Parent', fig);

for kk = 1:size(theFiles, 1)
    display(kk);
    load(theFiles(kk).name);
    % [S(kk).fail_traj, S(kk).fail_where] = ReachSet_CheckBound(R_total);
    non_empty_cells = find(~cellfun(@isempty, R_total));
    for jj = non_empty_cells'
        axes(ax1);
        hold on;
        plot(R_total{jj}.reachsets, [1 2], 'FaceColor', ax1.ColorOrder(ax1.ColorOrderIndex, :), 'FaceAlpha', .3, 'EdgeAlpha', .3); % ax1.ColorOrderIndex = mod(ax1.ColorOrderIndex + 1, 7) ;
        axes(ax2);
        hold on;
        plot(R_total{jj}.reachsets, [1 3], 'FaceColor', ax2.ColorOrder(ax2.ColorOrderIndex, :), 'FaceAlpha', .3, 'EdgeAlpha', .3); % ax2.ColorOrderIndex = ax2.ColorOrderIndex + 1;
        axes(ax3);
        hold on;
        plot(R_total{jj}.reachsets, [1 5], 'FaceColor', ax3.ColorOrder(ax3.ColorOrderIndex, :), 'FaceAlpha', .3, 'EdgeAlpha', .3); % ax3.ColorOrderIndex = ax3.ColorOrderIndex + 1;
        axes(ax4);
        hold on;
        plot(R_total{jj}.reachsets, [1 6], 'FaceColor', ax4.ColorOrder(ax4.ColorOrderIndex, :), 'FaceAlpha', .3, 'EdgeAlpha', .3); % ax3.ColorOrderIndex = ax3.ColorOrderIndex + 1;
        axes(ax5);
        hold on;
        plot(R_total{jj}.ref_traj_states(:, 1), R_total{jj}.ref_traj_states(:, 2));
        % axes(ax5); hold on; plot(R_total{jj}.ref_traj_states(:,1), -R_total{jj}.ref_traj_states(:,2));
        axes(ax6);
        hold on;
        plot(R_total{jj}.ref_traj_states(:, 3));
        axes(ax7);
        hold on;
        plot(R_total{jj}.ref_traj_states(:, 5));
        axes(ax8);
        hold on;
        plot(R_total{jj}.ref_traj_states(:, 6));
    end
end
