function sim = simulateRocket(f, alpha, cfgSI)
%SIMULATEROCKET 水噴射・空気噴射・慣性飛行の多フェーズ軌道を最高点まで積分する
%   成功時は status="success" と最高到達高度 Hmax・時系列データを返し、
%   飛行不能やタイムアウト時は対応する診断ステータスを返す (NaN単体は返さない)。

    sim = initEmptySim(f, alpha, cfgSI);

    % --- 1. 入力値 f, alpha の検証 ---
    if ~isnumeric(f) || ~isscalar(f) || ~isfinite(f) || f <= 0.0 || f >= 1.0
        sim.status = "invalidInput";
        sim.message = "Initial water fill fraction f must be a finite scalar in (0, 1).";
        return;
    end
    if ~isnumeric(alpha) || numel(alpha) ~= cfgSI.nControlKnots || any(~isfinite(alpha(:)))
        sim.status = "invalidInput";
        sim.message = "Control vector alpha has invalid length or non-finite values.";
        return;
    end
    if any(alpha(:) < -1e-6 | alpha(:) > 1.0 + 1e-6)
        sim.status = "invalidInput";
        sim.message = "Control knot values alpha must lie in [0, 1].";
        return;
    end
    alphaRow = min(max(alpha(:).', 0.0), 1.0);

    % --- 2. 初期圧力と離陸成立性の確認 ---
    if cfgSI.p0Abs <= cfgSI.pAmb
        sim.status = "noLiftoff";
        sim.message = "Initial tank pressure p0Abs does not exceed ambient pressure pAmb.";
        return;
    end

    Vw0 = f * cfgSI.Vb;
    Va0 = cfgSI.Vb - Vw0;
    ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
    y0 = [0.0; 0.0; 0.0; 0.0; Vw0; ma0];

    dy0 = wr.rocketRHS(0.0, y0, "water", alphaRow, cfgSI);
    if dy0(4) <= 0.0
        sim.status = "noLiftoff";
        sim.message = "Initial vertical acceleration is non-positive (thrust insufficient to lift off).";
        sim = populateSinglePointTrajectory(sim, 0.0, y0, alphaRow, cfgSI);
        return;
    end

    % --- 3. ODEソルバ設定 ---
    dtKnot = cfgSI.tControlEnd / max(cfgSI.nControlKnots - 1, 1);
    maxStepProp = min(cfgSI.maxStep, dtKnot / 4.0);
    maxStepCoast = cfgSI.maxStep;

    try
        % =====================================================================
        % PHASE W: 水噴射フェーズ
        % =====================================================================
        optsW = odeset( ...
            'RelTol', cfgSI.relTol, ...
            'AbsTol', cfgSI.absTol, ...
            'MaxStep', maxStepProp, ...
            'Events', @(t, y) wr.rocketEvents(t, y, "water", alphaRow, cfgSI));

        [tW, yW, teW, ~, ieW] = ode45( ...
            @(t, y) wr.rocketRHS(t, y, "water", alphaRow, cfgSI), ...
            [0.0, cfgSI.maxFlightTime], y0, optsW);

        % 微小な負の丸め誤差をゼロに補正
        yW(:, 5) = max(yW(:, 5), 0.0);
        yW(:, 6) = max(yW(:, 6), 0.0);

        tAll = tW;
        yAll = yW;
        phaseIdAll = ones(size(tW, 1), 1);

        tEndW = tW(end);
        yEndW = yW(end, :).';
        propEndW = wr.waterPropulsion(tEndW, yEndW, alphaRow, cfgSI);
        tPropEnd = tEndW;

        % Phase W 終了要因の判定 (残水・圧力条件を優先)
        ieWLast = getLastEventIndices(teW, ieW, tEndW);
        runPhaseA = false;
        runPhaseC = false;

        pCutTol = 2e-2;
        waterEmpty = any(ieWLast == 1) || (yEndW(5) <= 1e-9 * cfgSI.Vb);
        pressureCutW = any(ieWLast == 2) || (propEndW.p <= cfgSI.pAmb + pCutTol);
        apexInW = any(ieWLast == 3) && (tEndW > 1e-6) && (yEndW(2) > 1e-5);
        groundInW = any(ieWLast == 4) && (tEndW > 1e-6);

        if waterEmpty
            yEndW(5) = 0.0;
            yAll(end, 5) = 0.0;
            propEndW = wr.waterPropulsion(tEndW, yEndW, alphaRow, cfgSI);
            sim.phaseEvents.waterEnd = struct( ...
                't', tEndW, ...
                'reason', "waterDepleted", ...
                'VwRemaining', 0.0, ...
                'pEnd', propEndW.p, ...
                'yEnd', yEndW);
            sim.phaseEvents.list(end+1) = struct( ...
                'phase', "water", 'type', "waterDepleted", 't', tEndW, 'y', yEndW);

            if apexInW || yEndW(4) <= 0.0
                sim.phaseEvents.airEnd.reason = "skipped";
                sim.status = "success";
                sim.Hmax = yEndW(2);
                sim.tApex = tEndW;
                sim.phaseEvents.apex = struct('t', tEndW, 'z', yEndW(2), 'vx', yEndW(3), 'vz', yEndW(4), 'yEnd', yEndW);
            elseif propEndW.p > cfgSI.pAmb + pCutTol
                runPhaseA = true;
            else
                sim.phaseEvents.airEnd.reason = "skipped";
                runPhaseC = true;
            end

        elseif pressureCutW
            sim.phaseEvents.waterEnd = struct( ...
                't', tEndW, ...
                'reason', "pressureEqualized", ...
                'VwRemaining', yEndW(5), ...
                'pEnd', propEndW.p, ...
                'yEnd', yEndW);
            sim.phaseEvents.airEnd.reason = "skipped";
            sim.phaseEvents.list(end+1) = struct( ...
                'phase', "water", 'type', "pressureEqualized", 't', tEndW, 'y', yEndW);

            if apexInW || yEndW(4) <= 0.0
                sim.status = "success";
                sim.Hmax = yEndW(2);
                sim.tApex = tEndW;
                sim.phaseEvents.apex = struct('t', tEndW, 'z', yEndW(2), 'vx', yEndW(3), 'vz', yEndW(4), 'yEnd', yEndW);
            else
                runPhaseC = true;
            end

        elseif apexInW
            sim.phaseEvents.waterEnd = struct( ...
                't', tEndW, ...
                'reason', "apexReached", ...
                'VwRemaining', yEndW(5), ...
                'pEnd', propEndW.p, ...
                'yEnd', yEndW);
            sim.phaseEvents.airEnd.reason = "skipped";
            sim.phaseEvents.apex = struct('t', tEndW, 'z', yEndW(2), 'vx', yEndW(3), 'vz', yEndW(4), 'yEnd', yEndW);
            sim.phaseEvents.list(end+1) = struct( ...
                'phase', "water", 'type', "apexReached", 't', tEndW, 'y', yEndW);
            sim.status = "success";
            sim.Hmax = yEndW(2);
            sim.tApex = tEndW;

        elseif groundInW
            sim.phaseEvents.waterEnd = struct( ...
                't', tEndW, ...
                'reason', "groundImpact", ...
                'VwRemaining', yEndW(5), ...
                'pEnd', propEndW.p, ...
                'yEnd', yEndW);
            sim.phaseEvents.airEnd.reason = "skipped";
            if max(yAll(:, 2)) < 1e-3
                sim.status = "noLiftoff";
                sim.message = "Rocket failed to achieve sustained liftoff in Phase W.";
            else
                sim.status = "groundImpact";
                sim.message = "Rocket impacted ground during Phase W.";
            end

        else
            sim.phaseEvents.waterEnd = struct( ...
                't', tEndW, ...
                'reason', "timeout", ...
                'VwRemaining', yEndW(5), ...
                'pEnd', propEndW.p, ...
                'yEnd', yEndW);
            sim.phaseEvents.airEnd.reason = "skipped";
            sim.status = "timeout";
            sim.message = "Maximum flight time reached during Phase W.";
        end

        % =====================================================================
        % PHASE A: 圧縮空気噴射フェーズ
        % =====================================================================
        tCurr = tEndW;
        yCurr = yEndW;
        phase0 = struct( ...
            'p1', propEndW.p, ...
            'T1', propEndW.T, ...
            'ma1', yEndW(6), ...
            'tStart', tEndW, ...
            'yStart', yEndW);
        sim.phaseEvents.airStart = phase0;

        if runPhaseA
            if tCurr >= cfgSI.maxFlightTime
                sim.phaseEvents.airEnd = struct( ...
                    't', tCurr, 'reason', "timeout", 'maRemaining', yCurr(6), 'pEnd', propEndW.p, 'yEnd', yCurr);
                sim.status = "timeout";
                sim.message = "Maximum flight time reached at start of Phase A.";
            else
                optsA = odeset( ...
                    'RelTol', cfgSI.relTol, ...
                    'AbsTol', cfgSI.absTol, ...
                    'MaxStep', maxStepProp, ...
                    'Events', @(t, y) wr.rocketEvents(t, y, "air", alphaRow, cfgSI, phase0));

                [tA, yA, teA, ~, ieA] = ode45( ...
                    @(t, y) wr.rocketRHS(t, y, "air", alphaRow, cfgSI, phase0), ...
                    [tCurr, cfgSI.maxFlightTime], yCurr, optsA);

                yA(:, 5) = 0.0;
                yA(:, 6) = max(yA(:, 6), 0.0);

                if size(tA, 1) > 1
                    tAll = [tAll; tA(2:end)];
                    yAll = [yAll; yA(2:end, :)];
                    phaseIdAll = [phaseIdAll; 2 * ones(size(tA, 1) - 1, 1)];
                end

                tEndA = tA(end);
                yEndA = yA(end, :).';
                propEndA = wr.airPropulsion(tEndA, yEndA, alphaRow, cfgSI, phase0);
                tPropEnd = tEndA;
                tCurr = tEndA;
                yCurr = yEndA;

                ieALast = getLastEventIndices(teA, ieA, tEndA);
                pressureCutA = any(ieALast == 1) || (propEndA.p <= cfgSI.pAmb + pCutTol);
                apexInA = any(ieALast == 2) && (yEndA(2) > 1e-5);
                groundInA = any(ieALast == 3);

                if pressureCutA
                    sim.phaseEvents.airEnd = struct( ...
                        't', tEndA, ...
                        'reason', "pressureEqualized", ...
                        'maRemaining', yEndA(6), ...
                        'pEnd', propEndA.p, ...
                        'yEnd', yEndA);
                    sim.phaseEvents.list(end+1) = struct( ...
                        'phase', "air", 'type', "pressureEqualized", 't', tEndA, 'y', yEndA);

                    if apexInA || yEndA(4) <= 0.0
                        sim.status = "success";
                        sim.Hmax = yEndA(2);
                        sim.tApex = tEndA;
                        sim.phaseEvents.apex = struct('t', tEndA, 'z', yEndA(2), 'vx', yEndA(3), 'vz', yEndA(4), 'yEnd', yEndA);
                    else
                        runPhaseC = true;
                    end
                elseif apexInA
                    sim.phaseEvents.airEnd = struct( ...
                        't', tEndA, ...
                        'reason', "apexReached", ...
                        'maRemaining', yEndA(6), ...
                        'pEnd', propEndA.p, ...
                        'yEnd', yEndA);
                    sim.phaseEvents.apex = struct('t', tEndA, 'z', yEndA(2), 'vx', yEndA(3), 'vz', yEndA(4), 'yEnd', yEndA);
                    sim.phaseEvents.list(end+1) = struct( ...
                        'phase', "air", 'type', "apexReached", 't', tEndA, 'y', yEndA);
                    sim.status = "success";
                    sim.Hmax = yEndA(2);
                    sim.tApex = tEndA;
                elseif groundInA
                    sim.phaseEvents.airEnd = struct( ...
                        't', tEndA, ...
                        'reason', "groundImpact", ...
                        'maRemaining', yEndA(6), ...
                        'pEnd', propEndA.p, ...
                        'yEnd', yEndA);
                    sim.status = "groundImpact";
                    sim.message = "Rocket impacted ground during Phase A.";
                else
                    sim.phaseEvents.airEnd = struct( ...
                        't', tEndA, ...
                        'reason', "timeout", ...
                        'maRemaining', yEndA(6), ...
                        'pEnd', propEndA.p, ...
                        'yEnd', yEndA);
                    sim.status = "timeout";
                    sim.message = "Maximum flight time reached during Phase A.";
                end
            end
        end

        % =====================================================================
        % PHASE C: 慣性飛行フェーズ
        % =====================================================================
        if runPhaseC
            if tCurr >= cfgSI.maxFlightTime
                sim.status = "timeout";
                sim.message = "Maximum flight time reached before Phase C.";
            elseif yCurr(4) <= 0.0
                sim.status = "success";
                sim.Hmax = yCurr(2);
                sim.tApex = tCurr;
                sim.phaseEvents.apex = struct('t', tCurr, 'z', yCurr(2), 'vx', yCurr(3), 'vz', yCurr(4), 'yEnd', yCurr);
            else
                optsC = odeset( ...
                    'RelTol', cfgSI.relTol, ...
                    'AbsTol', cfgSI.absTol, ...
                    'MaxStep', maxStepCoast, ...
                    'Events', @(t, y) wr.rocketEvents(t, y, "coast", alphaRow, cfgSI, phase0));

                [tC, yC, teC, ~, ieC] = ode45( ...
                    @(t, y) wr.rocketRHS(t, y, "coast", alphaRow, cfgSI, phase0), ...
                    [tCurr, cfgSI.maxFlightTime], yCurr, optsC);

                yC(:, 5) = max(yC(:, 5), 0.0);
                yC(:, 6) = max(yC(:, 6), 0.0);

                if size(tC, 1) > 1
                    tAll = [tAll; tC(2:end)];
                    yAll = [yAll; yC(2:end, :)];
                    phaseIdAll = [phaseIdAll; 3 * ones(size(tC, 1) - 1, 1)];
                end

                tEndC = tC(end);
                yEndC = yC(end, :).';
                ieCLast = getLastEventIndices(teC, ieC, tEndC);

                if any(ieCLast == 1)
                    sim.status = "success";
                    sim.Hmax = yEndC(2);
                    sim.tApex = tEndC;
                    sim.phaseEvents.apex = struct('t', tEndC, 'z', yEndC(2), 'vx', yEndC(3), 'vz', yEndC(4), 'yEnd', yEndC);
                    sim.phaseEvents.list(end+1) = struct( ...
                        'phase', "coast", 'type', "apexReached", 't', tEndC, 'y', yEndC);
                elseif any(ieCLast == 2)
                    sim.status = "groundImpact";
                    sim.message = "Rocket impacted ground during Phase C.";
                else
                    sim.status = "timeout";
                    sim.message = "Maximum flight time reached during Phase C before apex.";
                end
            end
        end

        % --- 4. 全時刻の推力・圧力・開口面積履歴の再構築 ---
        sim = populateTrajectoryHistories(sim, tAll, yAll, phaseIdAll, alphaRow, cfgSI, phase0, tPropEnd);

    catch ME
        sim.status = "numericalFailure";
        sim.message = sprintf('ODE integration failed: %s', ME.message);
    end
end

function ieLast = getLastEventIndices(te, ie, tEnd)
    if isempty(te) || isempty(ie)
        ieLast = [];
        return;
    end
    mask = abs(te - tEnd) <= 1e-12 * max(1.0, abs(tEnd));
    ieLast = ie(mask);
end

function sim = initEmptySim(f, alpha, cfgSI)
    emptyEvent = struct('phase', "", 'type', "", 't', NaN, 'y', nan(6, 1));
    sim = struct( ...
        'status', "numericalFailure", ...
        'message', "", ...
        'f', f, ...
        'Vw0', f * cfgSI.Vb, ...
        'alpha', alpha(:).', ...
        'Hmax', 0.0, ...
        'tApex', NaN, ...
        'tPropulsionEnd', 0.0, ...
        'controlHoldActive', false, ...
        't', zeros(0, 1), ...
        'y', zeros(0, 6), ...
        'x', zeros(0, 1), ...
        'z', zeros(0, 1), ...
        'vx', zeros(0, 1), ...
        'vz', zeros(0, 1), ...
        'Vw', zeros(0, 1), ...
        'ma', zeros(0, 1), ...
        'mTot', zeros(0, 1), ...
        'p', zeros(0, 1), ...
        'T', zeros(0, 1), ...
        'thrust', zeros(0, 1), ...
        'A', zeros(0, 1), ...
        'dEq', zeros(0, 1), ...
        'phaseId', zeros(0, 1), ...
        'phaseEvents', struct( ...
            'waterEnd', struct('t', NaN, 'reason', "none", 'VwRemaining', NaN, 'pEnd', NaN, 'yEnd', nan(6, 1)), ...
            'airStart', struct('p1', NaN, 'T1', NaN, 'ma1', NaN, 'tStart', NaN, 'yStart', nan(6, 1)), ...
            'airEnd', struct('t', NaN, 'reason', "none", 'maRemaining', NaN, 'pEnd', NaN, 'yEnd', nan(6, 1)), ...
            'apex', struct('t', NaN, 'z', NaN, 'vx', NaN, 'vz', NaN, 'yEnd', nan(6, 1)), ...
            'list', emptyEvent([])));
end

function sim = populateSinglePointTrajectory(sim, t0, y0, alphaRow, cfgSI)
    prop0 = wr.waterPropulsion(t0, y0, alphaRow, cfgSI);
    sim.t = t0;
    sim.y = y0(:).';
    sim.x = y0(1);
    sim.z = y0(2);
    sim.vx = y0(3);
    sim.vz = y0(4);
    sim.Vw = y0(5);
    sim.ma = y0(6);
    sim.mTot = cfgSI.mDry + cfgSI.rhoW * y0(5) + y0(6);
    sim.p = prop0.p;
    sim.T = prop0.T;
    sim.thrust = prop0.thrust;
    sim.A = prop0.A;
    sim.dEq = prop0.dEq;
    sim.phaseId = 1;
end

function sim = populateTrajectoryHistories(sim, tAll, yAll, phaseIdAll, alphaRow, cfgSI, phase0, tPropEnd)
    nSteps = numel(tAll);
    pHist = zeros(nSteps, 1);
    THist = zeros(nSteps, 1);
    thrustHist = zeros(nSteps, 1);
    [AHist, dEqHist] = wr.openingAtTime(tAll, alphaRow, cfgSI);

    % 慣性飛行フェーズ記録用に推進終了時点の温度を保持
    TCoast = cfgSI.T0;

    for k = 1:nSteps
        tk = tAll(k);
        yk = yAll(k, :).';
        pid = phaseIdAll(k);
        if pid == 1
            prop = wr.waterPropulsion(tk, yk, alphaRow, cfgSI);
            pHist(k) = max(prop.p, cfgSI.pAmb);
            THist(k) = prop.T;
            thrustHist(k) = prop.thrust;
            TCoast = prop.T;
        elseif pid == 2
            prop = wr.airPropulsion(tk, yk, alphaRow, cfgSI, phase0);
            pHist(k) = max(prop.p, cfgSI.pAmb);
            THist(k) = prop.T;
            thrustHist(k) = prop.thrust;
            TCoast = prop.T;
        else
            pHist(k) = cfgSI.pAmb;
            THist(k) = TCoast;
            thrustHist(k) = 0.0;
        end
    end

    yAll(:, 2) = max(yAll(:, 2), 0.0);
    yAll(:, 5) = max(yAll(:, 5), 0.0);
    yAll(:, 6) = max(yAll(:, 6), 0.0);

    sim.t = tAll;
    sim.y = yAll;
    sim.x = yAll(:, 1);
    sim.z = yAll(:, 2);
    sim.vx = yAll(:, 3);
    sim.vz = yAll(:, 4);
    sim.Vw = yAll(:, 5);
    sim.ma = yAll(:, 6);
    sim.mTot = cfgSI.mDry + cfgSI.rhoW * sim.Vw + sim.ma;
    sim.p = pHist;
    sim.T = THist;
    sim.thrust = thrustHist;
    sim.A = AHist;
    sim.dEq = dEqHist;
    sim.phaseId = phaseIdAll;
    sim.tPropulsionEnd = tPropEnd;
    sim.controlHoldActive = (tPropEnd > cfgSI.tControlEnd);
end
