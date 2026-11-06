function figs = plotResults(result, cfgSI)
%PLOTRESULTS Plot optimal vs constant-nozzle baseline water rocket trajectories.
%   figs = wr.plotResults(result, cfgSI) generates an 8-panel overview figure
%   comparing the optimal variable-nozzle schedule against the best constant-
%   opening baseline in SI units:
%     1. Effective nozzle area A(t) [m^2]
%     2. Equivalent nozzle diameter dEq(t) [m]
%     3. Altitude history z(t) [m]
%     4. 2D flight trajectory z vs x [m]
%     5. Tank absolute pressure p(t) [Pa]
%     6. Thrust history F_T(t) [N]
%     7. Remaining water volume Vw(t) [m^3]
%     8. Velocity components vx(t), vz(t) [m/s]

    if nargin < 2 || isempty(cfgSI)
        cfgSI = result.cfgSI;
    end

    vis = 'on';
    if isfield(cfgSI, 'showPlots') && ~isempty(cfgSI.showPlots) && ~cfgSI.showPlots
        vis = 'off';
    elseif ~usejava('desktop')
        vis = 'off';
    end

    fig = figure('Name', 'Water Rocket Optimal Control Results', ...
        'NumberTitle', 'off', ...
        'Color', 'w', ...
        'Position', [80, 60, 1200, 880], ...
        'Visible', vis);

    sBest = result.best.sim;
    sBase = result.baseline.sim;

    tPropWindow = max([cfgSI.tControlEnd * 1.25, sBest.tPropulsionEnd * 1.15, sBase.tPropulsionEnd * 1.15, 0.05]);
    tNoz = linspace(0.0, tPropWindow, 250);
    [ABestNoz, dEqBestNoz] = wr.openingAtTime(tNoz, result.best.alpha, cfgSI);
    [ABaseNoz, dEqBaseNoz] = wr.openingAtTime(tNoz, result.baseline.alpha, cfgSI);

    % 1. Effective Nozzle Area A(t) [m^2]
    subplot(4, 2, 1, 'Parent', fig);
    hold on; grid on;
    plot(tNoz, ABaseNoz, '--', 'LineWidth', 1.4, 'DisplayName', 'Baseline (const)');
    plot(tNoz, ABestNoz, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal A*(t)');
    plot(result.best.tKnots, result.best.AKnots, 'o', 'MarkerSize', 5, 'DisplayName', 'Control knots');
    xlabel('Time t [s]');
    ylabel('Effective Area A(t) [m^2]');
    title('Nozzle Effective Opening Area A(t)');
    legend('Location', 'best');

    % 2. Equivalent Nozzle Diameter dEq(t) [m]
    subplot(4, 2, 2, 'Parent', fig);
    hold on; grid on;
    plot(tNoz, dEqBaseNoz, '--', 'LineWidth', 1.4, 'DisplayName', 'Baseline (const)');
    plot(tNoz, dEqBestNoz, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal d_{eq}^*(t)');
    plot(result.best.tKnots, result.best.dEqKnots, 'o', 'MarkerSize', 5, 'DisplayName', 'Control knots');
    xlabel('Time t [s]');
    ylabel('Eq. Diameter d_{eq}(t) [m]');
    title('Nozzle Equivalent Diameter d_{eq}(t)');
    legend('Location', 'best');

    % 3. Altitude z(t) [m]
    subplot(4, 2, 3, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.t)
        plot(sBase.t, sBase.z, '--', 'LineWidth', 1.4, 'DisplayName', ...
            sprintf('Baseline (H_{max}=%.2f m)', result.baseline.Hmax));
    end
    if ~isempty(sBest.t)
        plot(sBest.t, sBest.z, '-', 'LineWidth', 1.8, 'DisplayName', ...
            sprintf('Optimal (H_{max}=%.2f m)', result.best.Hmax));
    end
    xlabel('Time t [s]');
    ylabel('Altitude z(t) [m]');
    title('Altitude History z(t)');
    legend('Location', 'southeast');

    % 4. 2D Trajectory x(t) vs z(t) [m]
    subplot(4, 2, 4, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.x)
        plot(sBase.x, sBase.z, '--', 'LineWidth', 1.4, 'DisplayName', 'Baseline');
    end
    if ~isempty(sBest.x)
        plot(sBest.x, sBest.z, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal');
    end
    xlabel('Downrange x [m]');
    ylabel('Altitude z [m]');
    title('2D Flight Trajectory (Ascent to Apex)');
    legend('Location', 'southeast');

    % 5. Tank Absolute Pressure p(t) [Pa] (zoomed to propulsion window)
    subplot(4, 2, 5, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.t)
        plot(sBase.t, sBase.p, '--', 'LineWidth', 1.4, 'DisplayName', 'Baseline p(t)');
    end
    if ~isempty(sBest.t)
        plot(sBest.t, sBest.p, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal p(t)');
    end
    yline(cfgSI.pAmb, ':', 'p_{amb}', 'DisplayName', 'Ambient p_{amb}');
    xlim([0, tPropWindow]);
    xlabel('Time t [s]');
    ylabel('Tank Pressure p(t) [Pa]');
    title('Tank Absolute Pressure (Propulsion Phase)');
    legend('Location', 'northeast');

    % 6. Thrust F_T(t) [N] (zoomed to propulsion window)
    subplot(4, 2, 6, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.t)
        plot(sBase.t, sBase.thrust, '--', 'LineWidth', 1.4, 'DisplayName', 'Baseline F_T(t)');
    end
    if ~isempty(sBest.t)
        plot(sBest.t, sBest.thrust, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal F_T(t)');
    end
    xlim([0, tPropWindow]);
    xlabel('Time t [s]');
    ylabel('Thrust F_T(t) [N]');
    title('Thrust History (Propulsion Phase)');
    legend('Location', 'northeast');

    % 7. Remaining Water Volume Vw(t) [m^3] (zoomed to propulsion window)
    subplot(4, 2, 7, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.t)
        plot(sBase.t, sBase.Vw, '--', 'LineWidth', 1.4, 'DisplayName', ...
            sprintf('Baseline (f=%.2f)', result.baseline.f));
    end
    if ~isempty(sBest.t)
        plot(sBest.t, sBest.Vw, '-', 'LineWidth', 1.8, 'DisplayName', ...
            sprintf('Optimal (f^*=%.2f)', result.best.f));
    end
    xlim([0, tPropWindow]);
    xlabel('Time t [s]');
    ylabel('Water Volume V_w(t) [m^3]');
    title('Remaining Water Volume V_w(t)');
    legend('Location', 'northeast');

    % 8. Velocity Components vx(t), vz(t) [m/s]
    subplot(4, 2, 8, 'Parent', fig);
    hold on; grid on;
    if ~isempty(sBase.t)
        plot(sBase.t, sBase.vz, '--', 'LineWidth', 1.2, 'DisplayName', 'Baseline v_z');
        plot(sBase.t, sBase.vx, ':', 'LineWidth', 1.2, 'DisplayName', 'Baseline v_x');
    end
    if ~isempty(sBest.t)
        plot(sBest.t, sBest.vz, '-', 'LineWidth', 1.8, 'DisplayName', 'Optimal v_z');
        plot(sBest.t, sBest.vx, '-.', 'LineWidth', 1.5, 'DisplayName', 'Optimal v_x');
    end
    xlabel('Time t [s]');
    ylabel('Velocity [m/s]');
    title('Velocity Components v_x(t), v_z(t)');
    legend('Location', 'best');

    figs = struct('overview', fig);
end
