function void = Visualize_SingleR(R_total_inst)
    % VISUALIZE_SINGLER Summary of this function goes here
    %   Detailed explanation goes here

    len = length(R_total_inst.R0.Z(:, 1)) / 2;

    tiledlayout(len, 6);
    nexttile([len, 4]);
    hold on;
    L = plot(R_total_inst.RS_ext, [1 2], 'Facecolor', 'b');
    R1 = plot(R_total_inst.RS_ext(end).timePoint.set{end, 1}, [1 2], 'Facecolor', 'g', 'Edgecolor', 'k');
    R2 = plot(R_total_inst.RS_ext(1).timePoint.set{1, 1}, [1 2], 'Facecolor', 'w', 'Edgecolor', 'k');
    L2 = plot(R_total_inst.RS_ext, [7 8], 'r');
    xlabel('$X$-Coordinate [$m$]', 'Interpreter', 'latex');
    ylabel('$Y$-Coordinate [$m$]', 'Interpreter', 'latex');
    legend([L, R1, R2, L2], {'Reachable Set', 'End Set', 'Initial Set', 'Reference Trajectory'});

    nexttile([1 2]);
    hold on;
    title('$X$-$Y$-Coordinates', 'Interpreter', 'latex');
    R21 = plot(R_total_inst.R0, [1 2], 'Facecolor', 'w', 'Edgecolor', 'k');
    R22 = plot(R_total_inst.R_endshifted, [1 2], 'Facecolor', 'y', 'Edgecolor', 'g');

    nexttile([1 2]);
    hold on;
    title('$X$-$\delta$', 'Interpreter', 'latex');
    R31 = plot(R_total_inst.R0, [1 3], 'Facecolor', 'w', 'Edgecolor', 'k');
    R32 = plot(R_total_inst.R_endshifted, [1 3], 'Facecolor', 'y', 'Edgecolor', 'g');

    nexttile([1 2]);
    hold on;
    title('$X$-$v$', 'Interpreter', 'latex');
    R41 = plot(R_total_inst.R0, [1 4], 'Facecolor', 'w', 'Edgecolor', 'k');
    R42 = plot(R_total_inst.R_endshifted, [1 4], 'Facecolor', 'y', 'Edgecolor', 'g');

    nexttile([1 2]);
    hold on;
    title('$X$-$\Psi$', 'Interpreter', 'latex');
    R51 = plot(R_total_inst.R0, [1 5], 'Facecolor', 'w', 'Edgecolor', 'k');
    R52 = plot(R_total_inst.R_endshifted, [1 5], 'Facecolor', 'y', 'Edgecolor', 'g');

    nexttile([1 2]);
    hold on;
    title('$X$-$a$', 'Interpreter', 'latex');
    R61 = plot(R_total_inst.R0, [1 6], 'Facecolor', 'w', 'Edgecolor', 'k');
    R62 = plot(R_total_inst.R_endshifted, [1 6], 'Facecolor', 'y', 'Edgecolor', 'g');

end
