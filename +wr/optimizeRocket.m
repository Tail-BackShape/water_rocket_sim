function result = optimizeRocket(cfgSI)
%OPTIMIZEROCKET Optimize initial water fill fraction and variable nozzle schedule.
%   result = wr.optimizeRocket(cfgSI) performs:
%     1. Preliminary constant-opening baseline grid search over (f, alpha_const).
%     2. Multi-start constrained local optimization via fmincon (SQP) subject to
%        fill bounds, opening bounds, and linear adjacent-knot slew constraints.
%     3. Optional Global Optimization Toolbox (ga) search if cfgSI.useGA is true.
%     4. Safe adoption rule guaranteeing result.best.Hmax >= result.baseline.Hmax.

    cfgSI = wr.validateConfig(cfgSI);

    if exist('fmincon', 'file') ~= 2
        error('wr:missingToolbox', ...
            ['Optimization Toolbox (fmincon) is required for constrained nozzle ', ...
             'schedule optimization. Unconstrained penalty substitutes (e.g. fminsearch) ', ...
             'are disabled per specification to guarantee strict constraint satisfaction.']);
    end

    % Manage random seed for reproducibility without polluting caller RNG state
    prevRng = rng;
    cleanupRng = onCleanup(@() rng(prevRng));
    if isfield(cfgSI, 'randomSeed') && ~isempty(cfgSI.randomSeed)
        rng(cfgSI.randomSeed, 'twister');
    end

    N = cfgSI.nControlKnots;
    tKnots = linspace(0.0, cfgSI.tControlEnd, N);
    dt = cfgSI.tControlEnd / (N - 1);
    deltaA = cfgSI.Amax - cfgSI.Amin;
    dAlphaMax = cfgSI.ARateMax * dt / deltaA;

    % --- 1. Build variable bounds and linear slew-rate constraints ---
    lb = [cfgSI.fillMin; zeros(N, 1)];
    ub = [cfgSI.fillMax; ones(N, 1)];
    [Aineq, bineq] = buildSlewConstraints(N, dAlphaMax);

    % Evaluation counters and best-feasible tracker
    nOdeEval = 0;
    nOdeFail = 0;
    best = struct();

    % --- 2. Preliminary Constant-Nozzle Baseline Search ---
    fGrid = linspace(cfgSI.fillMin, cfgSI.fillMax, 5);
    if cfgSI.Amin > 0.0
        alphaConstGrid = [0.0, 0.25, 0.50, 0.75, 1.0];
    else
        alphaConstGrid = [0.20, 0.40, 0.60, 0.80, 1.0];
    end

    bestBaselineQ = [0.5 * (cfgSI.fillMin + cfgSI.fillMax), 0.5 * ones(1, N)];
    bestBaselineSim = [];
    bestBaselineHmax = -Inf;

    gridRecords = struct('f', {}, 'alphaConst', {}, 'Hmax', {}, 'status', {});

    for iF = 1:numel(fGrid)
        for iA = 1:numel(alphaConstGrid)
            qCand = [fGrid(iF), alphaConstGrid(iA) * ones(1, N)];
            [costCand, simCand] = trackedEval(qCand);
            gridRecords(end+1) = struct( ... %#ok<AGROW>
                'f', fGrid(iF), ...
                'alphaConst', alphaConstGrid(iA), ...
                'Hmax', simCand.Hmax, ...
                'status', string(simCand.status));

            if costCand < 0.0 && simCand.Hmax > bestBaselineHmax
                bestBaselineHmax = simCand.Hmax;
                bestBaselineQ = qCand;
                bestBaselineSim = simCand;
            end
        end
    end

    if isempty(bestBaselineSim)
        % Evaluate default midpoint if no grid point lifted off
        [~, bestBaselineSim] = trackedEval(bestBaselineQ);
        bestBaselineHmax = max(bestBaselineSim.Hmax, 0.0);
    end

    baseline = packCandidateStruct(bestBaselineQ, bestBaselineSim, tKnots, cfgSI);
    baseline.gridRecords = gridRecords;

    % Initialize adopted best solution to the baseline (guarantees best >= baseline)
    best = baseline;

    % --- 3. Multi-Start Constrained Optimization with fmincon (SQP) ---
    nMultiStart = 3;
    if isfield(cfgSI, 'nMultiStart') && ~isempty(cfgSI.nMultiStart)
        nMultiStart = max(1, round(cfgSI.nMultiStart));
    end
    maxIter = 40;
    if isfield(cfgSI, 'maxIter') && ~isempty(cfgSI.maxIter)
        maxIter = max(5, round(cfgSI.maxIter));
    end

    initGuesses = generateInitialGuesses(bestBaselineQ, nMultiStart, lb, ub, dAlphaMax);

    fminconOpts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'off', ...
        'MaxIterations', maxIter, ...
        'MaxFunctionEvaluations', maxIter * (N + 3) * 3, ...
        'FiniteDifferenceStepSize', 1e-3, ...
        'StepTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-4, ...
        'ConstraintTolerance', 1e-8);

    bestExitFlag = 0;
    solverUsed = "fmincon-sqp";

    for iStart = 1:size(initGuesses, 1)
        q0 = initGuesses(iStart, :).';
        try
            [qOpt, ~, exitflag] = fmincon( ...
                @(q) objectiveOnly(q), ...
                q0, Aineq, bineq, [], [], lb, ub, [], fminconOpts);

            qOptFeas = projectToFeasible(qOpt(:).', lb, ub, dAlphaMax);
            [violNorm, ~] = computeConstraintViolation(qOptFeas, lb, ub, Aineq, bineq, tKnots, cfgSI);

            if violNorm <= 1e-6
                [costOpt, simOpt] = trackedEval(qOptFeas);
                if costOpt < 0.0 && simOpt.Hmax > best.Hmax
                    best = packCandidateStruct(qOptFeas, simOpt, tKnots, cfgSI);
                    bestExitFlag = exitflag;
                elseif iStart == 1 && bestExitFlag == 0
                    bestExitFlag = exitflag;
                end
            end
        catch
            % Keep searching remaining starts; baseline remains safe fallback
        end
    end

    % --- 4. Optional Global Optimization Toolbox (ga) Search ---
    if isfield(cfgSI, 'useGA') && cfgSI.useGA
        if exist('ga', 'file') == 2
            try
                gaOpts = optimoptions('ga', ...
                    'Display', 'off', ...
                    'PopulationSize', max(20, 4 * (N + 1)), ...
                    'MaxGenerations', 15);
                [qGA, ~] = ga(@(q) objectiveOnly(q), N + 1, Aineq, bineq, [], [], lb.', ub.', [], gaOpts);
                qGAFeas = projectToFeasible(qGA(:).', lb, ub, dAlphaMax);
                [violGA, ~] = computeConstraintViolation(qGAFeas, lb, ub, Aineq, bineq, tKnots, cfgSI);
                if violGA <= 1e-6
                    [costGA, simGA] = trackedEval(qGAFeas);
                    solverUsed = "ga+fmincon-sqp";
                    if costGA < 0.0 && simGA.Hmax > best.Hmax
                        best = packCandidateStruct(qGAFeas, simGA, tKnots, cfgSI);
                    end
                end
            catch
                % Ignore optional GA error and retain best feasible solution
            end
        else
            warning('wr:gaUnavailable', ...
                'Global Optimization Toolbox (ga) is not installed; proceeding with multi-start fmincon.');
        end
    end

    % --- 5. Assemble Final Result & Diagnostics ---
    [maxViolNorm, maxSlewRate] = computeConstraintViolation(best.q, lb, ub, Aineq, bineq, tKnots, cfgSI);

    comparison = computeComparison(best.Hmax, baseline.Hmax);

    limitationsNote = [ ...
        "1. Point-mass 2D trajectory with ideal velocity-aligned thrust; pitch/yaw stability and aerodynamic flutter are not modeled."; ...
        "2. Nozzle schedule A(t) represents effective flow area; actual actuator force/torque, valve delay under pressure load, and opening-dependent discharge coefficients require hardware bench calibration."; ...
        "3. Simulation does not verify pressure vessel or joint burst safety; the user must independently confirm safe working pressure for the bottle and valve hardware."];

    diagnostics = struct( ...
        'maxConstraintViolation', maxViolNorm, ...
        'maxSlewRate', maxSlewRate, ...
        'ARateMaxLimit', cfgSI.ARateMax, ...
        'exitflag', bestExitFlag, ...
        'converged', (bestExitFlag > 0) || (best.Hmax >= baseline.Hmax && baseline.Hmax > 0), ...
        'solverUsed', solverUsed, ...
        'nOdeEvaluations', nOdeEval, ...
        'nOdeFailures', nOdeFail, ...
        'nControlKnots', N, ...
        'tControlEnd', cfgSI.tControlEnd, ...
        'tPropulsionEnd', best.sim.tPropulsionEnd, ...
        'controlHoldActive', best.sim.controlHoldActive, ...
        'limitationsNote', limitationsNote);

    inputSummary = struct( ...
        'Vb', cfgSI.Vb, ...
        'Vb_L', cfgSI.Vb * 1e3, ...
        'mDry', cfgSI.mDry, ...
        'D', cfgSI.D, ...
        'D_mm', cfgSI.D * 1e3, ...
        'p0Abs', cfgSI.p0Abs, ...
        'pGauge', cfgSI.pGauge, ...
        'pGauge_kPa', cfgSI.pGauge * 1e-3, ...
        'pAmb', cfgSI.pAmb, ...
        'theta0', cfgSI.theta0, ...
        'theta0_deg', rad2deg(cfgSI.theta0), ...
        'Amin', cfgSI.Amin, ...
        'Amax', cfgSI.Amax, ...
        'ARateMax', cfgSI.ARateMax, ...
        'tControlEnd', cfgSI.tControlEnd, ...
        'nControlKnots', cfgSI.nControlKnots);

    result = struct( ...
        'best', best, ...
        'baseline', baseline, ...
        'comparison', comparison, ...
        'phaseEvents', best.sim.phaseEvents, ...
        'diagnostics', diagnostics, ...
        'inputSummary', inputSummary, ...
        'cfgSI', cfgSI);

    % --- Nested helper functions ---
    function [c, s] = trackedEval(qVec)
        nOdeEval = nOdeEval + 1;
        [c, s] = wr.evaluateCandidate(qVec, cfgSI);
        if string(s.status) ~= "success"
            nOdeFail = nOdeFail + 1;
        else
            % Also check if any intermediate feasible evaluation beat `best`
            qRowCand = qVec(:).';
            [vNormCand, ~] = computeConstraintViolation(qRowCand, lb, ub, Aineq, bineq, tKnots, cfgSI);
            if vNormCand <= 1e-6 && isfield(best, 'Hmax') && s.Hmax > best.Hmax
                best = packCandidateStruct(qRowCand, s, tKnots, cfgSI);
            end
        end
    end

    function c = objectiveOnly(qVec)
        [c, ~] = trackedEval(qVec);
    end
end

function [Aineq, bineq] = buildSlewConstraints(N, dAlphaMax)
    % For i = 1 .. N-1:
    %   alpha_{i+1} - alpha_i <= dAlphaMax
    %   alpha_i - alpha_{i+1} <= dAlphaMax
    nConstraints = 2 * (N - 1);
    Aineq = zeros(nConstraints, N + 1);
    bineq = dAlphaMax * ones(nConstraints, 1);

    row = 0;
    for i = 1:(N - 1)
        row = row + 1;
        Aineq(row, i + 1) = -1.0;
        Aineq(row, i + 2) = 1.0;

        row = row + 1;
        Aineq(row, i + 1) = 1.0;
        Aineq(row, i + 2) = -1.0;
    end
end

function guesses = generateInitialGuesses(qBase, nStarts, lb, ub, dAlphaMax)
    N = numel(qBase) - 1;
    guesses = zeros(nStarts, N + 1);
    guesses(1, :) = qBase(:).';

    if nStarts >= 2
        % Start 2: Ramp from smaller opening to larger opening (thrust modulation)
        alphaRampUp = zeros(1, N);
        alphaRampUp(1) = max(0.0, qBase(2) - 0.5 * dAlphaMax);
        for k = 2:N
            alphaRampUp(k) = min(1.0, alphaRampUp(k - 1) + 0.8 * dAlphaMax);
        end
        guesses(2, :) = [qBase(1), alphaRampUp];
    end

    if nStarts >= 3
        % Start 3: Ramp from larger opening to smaller opening
        alphaRampDown = zeros(1, N);
        alphaRampDown(1) = min(1.0, qBase(2) + 0.5 * dAlphaMax);
        for k = 2:N
            alphaRampDown(k) = max(0.0, alphaRampDown(k - 1) - 0.8 * dAlphaMax);
        end
        guesses(3, :) = [qBase(1), alphaRampDown];
    end

    for s = 4:nStarts
        fRand = lb(1) + (ub(1) - lb(1)) * rand();
        alphaRand = zeros(1, N);
        alphaRand(1) = rand();
        for k = 2:N
            step = (2.0 * rand() - 1.0) * 0.9 * dAlphaMax;
            alphaRand(k) = min(max(alphaRand(k - 1) + step, 0.0), 1.0);
        end
        guesses(s, :) = [fRand, alphaRand];
    end
end

function qFeas = projectToFeasible(qRow, lb, ub, dAlphaMax)
    qFeas = min(max(qRow(:).', lb(:).'), ub(:).');
    N = numel(qFeas) - 1;
    % Forward-backward slew projection onto |alpha_{k+1} - alpha_k| <= dAlphaMax
    for k = 2:N
        lo = max(0.0, qFeas(k) - dAlphaMax);
        hi = min(1.0, qFeas(k) + dAlphaMax);
        qFeas(k + 1) = min(max(qFeas(k + 1), lo), hi);
    end
end

function [maxViolNorm, maxSlewRate] = computeConstraintViolation(qRow, lb, ub, Aineq, bineq, tKnots, cfgSI)
    qCol = qRow(:);
    violBounds = max([0.0; lb - qCol; qCol - ub]);
    violLinear = max([0.0; Aineq * qCol - bineq]);

    alpha = qRow(2:end);
    [AKnots, ~] = wr.openingAtTime(tKnots, alpha, cfgSI);
    slewRates = abs(diff(AKnots)) ./ diff(tKnots);
    maxSlewRate = max(slewRates);
    violSlewPhys = max(0.0, maxSlewRate - cfgSI.ARateMax) / max(cfgSI.ARateMax, 1e-12);

    maxViolNorm = max([violBounds, violLinear, violSlewPhys]);
end

function comp = computeComparison(bestHmax, baseHmax)
    impAbs = bestHmax - baseHmax;
    if isfinite(baseHmax) && baseHmax > 0.0
        impPct = 100.0 * impAbs / baseHmax;
    else
        impPct = NaN;
    end
    comp = struct( ...
        'bestHmax', bestHmax, ...
        'baselineHmax', baseHmax, ...
        'improvementAbs', impAbs, ...
        'improvementPct', impPct);
end

function cand = packCandidateStruct(qRow, sim, tKnots, cfgSI)
    qRow = qRow(:).';
    f = qRow(1);
    alpha = qRow(2:end);
    [AKnots, dEqKnots] = wr.openingAtTime(tKnots, alpha, cfgSI);
    cand = struct( ...
        'q', qRow, ...
        'f', f, ...
        'Vw0', f * cfgSI.Vb, ...
        'Vw0_L', f * cfgSI.Vb * 1e3, ...
        'alpha', alpha, ...
        'tKnots', tKnots, ...
        'AKnots', AKnots, ...
        'dEqKnots', dEqKnots, ...
        'Hmax', sim.Hmax, ...
        'tApex', sim.tApex, ...
        'sim', sim);
end
