function fig = animateResults(result, cfgSI)
%ANIMATERESULTS 2次元飛行軌道とノズル有効径の変化をアニメーション表示する
%   左パネル  : 2D軌道 (x vs z)、推力プルーム、リアルタイム飛行状態表示
%   右上パネル: ノズル開口断面 (dEq(t) と最小/最大径の比較)
%   右下パネル: ノズル等価直径スケジュール dEq(t) と現在時刻カーソル

    if nargin < 2 || isempty(cfgSI)
        cfgSI = result.cfgSI;
    end

    isInteractive = true;
    if isfield(cfgSI, 'showPlots') && ~isempty(cfgSI.showPlots) && ~cfgSI.showPlots
        isInteractive = false;
    elseif ~usejava('desktop')
        isInteractive = false;
    end

    vis = 'on';
    if ~isInteractive
        vis = 'off';
    end

    fig = figure('Name', 'Water Rocket Trajectory & Nozzle Animation', ...
        'NumberTitle', 'off', ...
        'Color', 'w', ...
        'Position', [100, 80, 1180, 680], ...
        'Visible', vis);

    sBest = result.best.sim;
    sBase = result.baseline.sim;

    if isempty(sBest.t) || numel(sBest.t) < 2
        return;
    end

    % --- 推進フェーズ (スロー再生) と慣性飛行フェーズの2段階タイムライン作成 ---
    tEndAll = sBest.t(end);
    tPropWindow = min(max([cfgSI.tControlEnd, sBest.tPropulsionEnd, 0.05]), tEndAll);

    if isInteractive
        nFramesProp = 55;
        nFramesCoast = 45;
    else
        % テスト・バッチ実行時は最小フレーム数で高速検証
        nFramesProp = 4;
        nFramesCoast = 4;
    end

    if tPropWindow < tEndAll
        tAnimProp = linspace(0.0, tPropWindow, nFramesProp);
        tAnimCoast = linspace(tPropWindow, tEndAll, nFramesCoast + 1);
        tAnim = [tAnimProp, tAnimCoast(2:end)];
    else
        tAnim = linspace(0.0, tEndAll, nFramesProp);
    end

    % アニメーション時刻への状態補間
    xAnim = interp1(sBest.t, sBest.x, tAnim, 'linear', 'extrap');
    zAnim = interp1(sBest.t, sBest.z, tAnim, 'linear', 'extrap');
    vxAnim = interp1(sBest.t, sBest.vx, tAnim, 'linear', 'extrap');
    vzAnim = interp1(sBest.t, sBest.vz, tAnim, 'linear', 'extrap');
    VwAnim = interp1(sBest.t, sBest.Vw, tAnim, 'linear', 'extrap');
    pAnim = interp1(sBest.t, sBest.p, tAnim, 'linear', 'extrap');
    thrustAnim = interp1(sBest.t, sBest.thrust, tAnim, 'linear', 'extrap');
    [AAnim, dEqAnim] = wr.openingAtTime(tAnim, result.best.alpha, cfgSI);

    dMin_mm = sqrt(4.0 * cfgSI.Amin / pi) * 1e3;
    dMax_mm = sqrt(4.0 * cfgSI.Amax / pi) * 1e3;
    dEqAnim_mm = dEqAnim * 1e3;

    tWaterEnd = result.phaseEvents.waterEnd.t;
    tAirEnd = result.phaseEvents.airEnd.t;

    % =========================================================================
    % パネル 1 (左): 2次元軌道 x(t) vs z(t)
    % =========================================================================
    axTraj = subplot(2, 2, [1, 3], 'Parent', fig);
    hold(axTraj, 'on');
    grid(axTraj, 'on');

    if ~isempty(sBase.x)
        plot(axTraj, sBase.x, sBase.z, '--', 'Color', [0.6, 0.6, 0.6], ...
            'LineWidth', 1.2, 'DisplayName', 'Baseline trajectory');
    end
    plot(axTraj, sBest.x, sBest.z, ':', 'Color', [0.75, 0.82, 0.95], ...
        'LineWidth', 1.2, 'DisplayName', 'Optimal full path');

    hTrail = plot(axTraj, xAnim(1), zAnim(1), '-', 'Color', [0.0, 0.45, 0.74], ...
        'LineWidth', 2.2, 'DisplayName', 'Flight trail');
    hPlume = plot(axTraj, [xAnim(1), xAnim(1)], [zAnim(1), zAnim(1)], '-', ...
        'Color', [0.93, 0.40, 0.10], 'LineWidth', 3.0, 'DisplayName', 'Jet plume');
    hRocket = plot(axTraj, xAnim(1), zAnim(1), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.85, 0.15, 0.15], ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Rocket');

    maxX = max([max(sBest.x), max(sBase.x), 1.0]);
    maxZ = max([max(sBest.z), max(sBase.z), 1.0]);
    spanPlot = max(maxX, maxZ) * 1.12;
    xlim(axTraj, [-0.05 * spanPlot, max(maxX * 1.25, 0.25 * spanPlot)]);
    ylim(axTraj, [0.0, maxZ * 1.12]);
    xlabel(axTraj, 'Downrange x [m]');
    ylabel(axTraj, 'Altitude z [m]');
    title(axTraj, '2D Flight Trajectory & Propulsion State');
    legend(axTraj, 'Location', 'northwest');

    hInfoText = text(axTraj, 0.03, 0.78, '', 'Units', 'normalized', ...
        'FontName', 'FixedWidth', 'FontSize', 9, ...
        'BackgroundColor', [0.97, 0.97, 0.97], 'EdgeColor', [0.8, 0.8, 0.8]);

    % =========================================================================
    % パネル 2 (右上): ノズル開口断面ビュー
    % =========================================================================
    axNozView = subplot(2, 2, 2, 'Parent', fig);
    hold(axNozView, 'on');
    axis(axNozView, 'equal');
    grid(axNozView, 'on');

    thetaCircle = linspace(0.0, 2.0 * pi, 120);
    rMax = max(dMax_mm / 2.0, 1.0);
    rMin = dMin_mm / 2.0;
    rInit = dEqAnim_mm(1) / 2.0;

    hNozPatch = patch(axNozView, rInit * cos(thetaCircle), rInit * sin(thetaCircle), ...
        [0.20, 0.60, 0.95], 'FaceAlpha', 0.55, 'EdgeColor', [0.05, 0.25, 0.55], ...
        'LineWidth', 2.0, 'DisplayName', 'Active opening d_{eq}(t)');
    plot(axNozView, rMax * cos(thetaCircle), rMax * sin(thetaCircle), '--', ...
        'Color', [0.3, 0.3, 0.3], 'LineWidth', 1.3, 'DisplayName', sprintf('d_{max} = %.1f mm', dMax_mm));
    if rMin > 0
        plot(axNozView, rMin * cos(thetaCircle), rMin * sin(thetaCircle), ':', ...
            'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.3, 'DisplayName', sprintf('d_{min} = %.1f mm', dMin_mm));
    end

    limR = rMax * 1.35;
    xlim(axNozView, [-limR, limR]);
    ylim(axNozView, [-limR, limR]);
    xlabel(axNozView, 'x_{nozzle} [mm]');
    ylabel(axNozView, 'y_{nozzle} [mm]');
    hNozTitle = title(axNozView, sprintf('Nozzle Orifice: d_{eq} = %.2f mm', dEqAnim_mm(1)));
    legend(axNozView, 'Location', 'northeast');

    % =========================================================================
    % パネル 3 (右下): ノズル等価直径スケジュール dEq(t) [mm]
    % =========================================================================
    axSched = subplot(2, 2, 4, 'Parent', fig);
    hold(axSched, 'on');
    grid(axSched, 'on');

    tSchedPlotMax = max([cfgSI.tControlEnd * 1.25, sBest.tPropulsionEnd * 1.25, 0.05]);
    tSchedGrid = linspace(0.0, tSchedPlotMax, 250);
    [~, dEqBestSched] = wr.openingAtTime(tSchedGrid, result.best.alpha, cfgSI);
    [~, dEqBaseSched] = wr.openingAtTime(tSchedGrid, result.baseline.alpha, cfgSI);

    plot(axSched, tSchedGrid, dEqBaseSched * 1e3, '--', 'Color', [0.55, 0.55, 0.55], ...
        'LineWidth', 1.3, 'DisplayName', 'Baseline d_{eq}');
    plot(axSched, tSchedGrid, dEqBestSched * 1e3, '-', 'Color', [0.0, 0.45, 0.74], ...
        'LineWidth', 1.8, 'DisplayName', 'Optimal d_{eq}^*(t)');
    plot(axSched, result.best.tKnots, result.best.dEqKnots * 1e3, 'o', ...
        'Color', [0.0, 0.45, 0.74], 'MarkerSize', 5, 'DisplayName', 'Control knots');

    yPad = max(0.1 * (dMax_mm - dMin_mm), 1.0);
    ylim(axSched, [max(0.0, dMin_mm - yPad), dMax_mm + yPad]);
    xlim(axSched, [0.0, tSchedPlotMax]);

    hCursorLine = plot(axSched, [0.0, 0.0], ylim(axSched), 'r-', ...
        'LineWidth', 1.4, 'DisplayName', 'Current time t');
    hCursorDot = plot(axSched, 0.0, dEqAnim_mm(1), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 7, 'HandleVisibility', 'off');

    xlabel(axSched, 'Time t [s] (Propulsion Window)');
    ylabel(axSched, 'Eq. Diameter d_{eq}(t) [mm]');
    title(axSched, 'Nozzle Diameter Schedule d_{eq}(t)');
    legend(axSched, 'Location', 'best');

    % =========================================================================
    % アニメーション更新ループ
    % =========================================================================
    plumeRefScale = 0.08 * maxZ / max(max(thrustAnim), 1.0);

    for k = 1:numel(tAnim)
        if ~isgraphics(fig)
            break;
        end

        tk = tAnim(k);
        xk = xAnim(k);
        zk = zAnim(k);
        vxk = vxAnim(k);
        vzk = vzAnim(k);
        vk = hypot(vxk, vzk);
        Fk = thrustAnim(k);
        Vwk = VwAnim(k);
        pk = pAnim(k);
        dEqk_mm = dEqAnim_mm(k);
        Ak = AAnim(k);

        % 現在の飛行フェーズと表示色の判定
        if isfinite(tWaterEnd) && tk <= tWaterEnd + 1e-6
            phaseLabel = 'Phase W (Water Jet)';
            nozColor = [0.15, 0.55, 0.95];
            plumeColor = [0.15, 0.55, 0.95];
            speedTag = '[SLOW-MO Propulsion]';
        elseif isfinite(tAirEnd) && tk <= tAirEnd + 1e-6 && Fk > 0
            phaseLabel = 'Phase A (Air Blowdown)';
            nozColor = [0.25, 0.80, 0.85];
            plumeColor = [0.95, 0.50, 0.15];
            speedTag = '[SLOW-MO Propulsion]';
        else
            phaseLabel = 'Phase C (Coast to Apex)';
            nozColor = [0.70, 0.70, 0.70];
            plumeColor = [0.70, 0.70, 0.70];
            speedTag = '[Real-time Coast]';
        end

        % 軌跡・機体位置・推力プルームの更新
        set(hTrail, 'XData', xAnim(1:k), 'YData', zAnim(1:k));
        set(hRocket, 'XData', xk, 'YData', zk);

        if Fk > 1e-2
            if vk < cfgSI.vEpsilon
                dirX = cos(cfgSI.theta0);
                dirZ = sin(cfgSI.theta0);
            else
                dirX = vxk / vk;
                dirZ = vzk / vk;
            end
            plumeLen = Fk * plumeRefScale;
            set(hPlume, 'XData', [xk, xk - plumeLen * dirX], ...
                        'YData', [zk, zk - plumeLen * dirZ], ...
                        'Color', plumeColor);
        else
            set(hPlume, 'XData', [xk, xk], 'YData', [zk, zk]);
        end

        % 状態テキストの更新
        infoStr = sprintf([ ...
            '%s\n', ...
            'Phase  : %s\n', ...
            't      = %6.3f s\n', ...
            'z      = %6.2f m  (x = %5.2f m)\n', ...
            '|v|    = %6.2f m/s\n', ...
            'Thrust = %6.2f N\n', ...
            'p_tank = %6.1f kPaG\n', ...
            'V_w    = %6.3f L'], ...
            speedTag, phaseLabel, tk, zk, xk, vk, Fk, ...
            max(pk - cfgSI.pAmb, 0.0) * 1e-3, Vwk * 1e3);
        set(hInfoText, 'String', infoStr);

        % ノズル断面円の更新
        rk = dEqk_mm / 2.0;
        set(hNozPatch, 'XData', rk * cos(thetaCircle), ...
                       'YData', rk * sin(thetaCircle), ...
                       'FaceColor', nozColor);
        set(hNozTitle, 'String', sprintf('Nozzle Orifice: d_{eq}(t) = %.2f mm (A = %.2e m^2)', dEqk_mm, Ak));

        % スケジュール上の時刻カーソル更新
        tCurPlot = min(tk, tSchedPlotMax);
        set(hCursorLine, 'XData', [tCurPlot, tCurPlot]);
        set(hCursorDot, 'XData', tCurPlot, 'YData', dEqk_mm);

        if isInteractive
            drawnow;
            pause(0.025);
        end
    end
end
