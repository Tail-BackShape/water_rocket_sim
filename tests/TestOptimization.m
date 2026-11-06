classdef TestOptimization < matlab.unittest.TestCase
    %TESTOPTIMIZATION Unit tests for wr.evaluateCandidate and wr.optimizeRocket

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testCandidateBounds(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);

            qValid = [0.35, 0.5 * ones(1, cfgSI.nControlKnots)];
            [cost, sim] = wr.evaluateCandidate(qValid, cfgSI);
            testCase.verifyTrue(isfinite(cost));
            testCase.verifyFalse(isnan(cost));
            testCase.verifyEqual(string(sim.status), "success");
            testCase.verifyEqual(cost, -sim.Hmax, 'AbsTol', 1e-12);

            res = wr.optimizeRocket(cfgSI);
            testCase.verifyGreaterThanOrEqual(res.best.f, cfgSI.fillMin - 1e-9);
            testCase.verifyLessThanOrEqual(res.best.f, cfgSI.fillMax + 1e-9);
            testCase.verifyGreaterThanOrEqual(res.best.alpha, -1e-9);
            testCase.verifyLessThanOrEqual(res.best.alpha, 1.0 + 1e-9);
        end

        function testSlewFeasibility(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            tKnots = linspace(0, cfgSI.tControlEnd, cfgSI.nControlKnots);
            [AKnots, ~] = wr.openingAtTime(tKnots, res.best.alpha, cfgSI);
            rates = abs(diff(AKnots)) ./ diff(tKnots);
            testCase.verifyLessThanOrEqual(rates, cfgSI.ARateMax + 1e-8);
            testCase.verifyLessThanOrEqual(res.diagnostics.maxConstraintViolation, 1e-6);
        end

        function testOptimizesWaterFraction(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);

            alpha = 0.5 * ones(1, cfgSI.nControlKnots);
            [c1, ~] = wr.evaluateCandidate([0.15, alpha], cfgSI);
            [c2, ~] = wr.evaluateCandidate([0.45, alpha], cfgSI);
            testCase.verifyNotEqual(c1, c2);

            res = wr.optimizeRocket(cfgSI);
            testCase.verifyEqual(numel(res.best.q), 1 + cfgSI.nControlKnots);
            testCase.verifyEqual(res.best.q(1), res.best.f, 'AbsTol', 1e-12);
        end

        function testNeverBelowBaseline(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            testCase.verifyEqual(string(res.best.sim.status), "success");
            testCase.verifyEqual(string(res.baseline.sim.status), "success");
            testCase.verifyGreaterThanOrEqual(res.best.Hmax, res.baseline.Hmax - 1e-12);
        end

        function testInvalidCandidateIsNotOptimal(testCase)
            cfg = fastOptConfig();
            cfg.Amin = 0.0;
            cfgSI = wr.validateConfig(cfg);

            % Closed nozzle candidate -> cannot lift off, must return finite penalty > 0 (never NaN)
            qClosed = [0.35, zeros(1, cfgSI.nControlKnots)];
            [costClosed, simClosed] = wr.evaluateCandidate(qClosed, cfgSI);
            testCase.verifyFalse(isnan(costClosed));
            testCase.verifyTrue(isfinite(costClosed));
            testCase.verifyGreaterThan(costClosed, 0.0);
            testCase.verifyNotEqual(string(simClosed.status), "success");
        end

        function testRepeatabilityWithSeed(testCase)
            cfg = fastOptConfig();
            cfg.randomSeed = 12345;
            cfgSI = wr.validateConfig(cfg);

            res1 = wr.optimizeRocket(cfgSI);
            res2 = wr.optimizeRocket(cfgSI);
            testCase.verifyEqual(res1.best.Hmax, res2.best.Hmax, 'AbsTol', 1e-9);
            testCase.verifyEqual(res1.best.q, res2.best.q, 'AbsTol', 1e-9);
        end

        function testResultContainsPhaseEvents(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            testCase.verifyTrue(isfield(res, 'phaseEvents'));
            testCase.verifyTrue(isfield(res.phaseEvents, 'waterEnd'));
            testCase.verifyTrue(isfield(res.phaseEvents, 'airEnd'));
            testCase.verifyTrue(isfield(res.phaseEvents, 'apex'));
            testCase.verifyTrue(isfinite(res.phaseEvents.waterEnd.t));
            testCase.verifyTrue(isfinite(res.phaseEvents.apex.t));
            testCase.verifyNotEqual(string(res.phaseEvents.waterEnd.reason), "none");
        end

        function testReportHasConstraintViolation(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            testCase.verifyTrue(isfield(res, 'diagnostics'));
            testCase.verifyTrue(isfield(res.diagnostics, 'maxConstraintViolation'));
            testCase.verifyTrue(isfinite(res.diagnostics.maxConstraintViolation));
            testCase.verifyLessThanOrEqual(res.diagnostics.maxConstraintViolation, 1e-6);
            testCase.verifyTrue(isfield(res.diagnostics, 'limitationsNote'));
            testCase.verifyNotEmpty(res.diagnostics.limitationsNote);
        end

        function testBaselineComparisonFinite(testCase)
            cfg = fastOptConfig();
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            testCase.verifyTrue(isfield(res, 'comparison'));
            testCase.verifyTrue(isfinite(res.comparison.bestHmax));
            testCase.verifyTrue(isfinite(res.comparison.baselineHmax));
            testCase.verifyTrue(isfinite(res.comparison.improvementAbs));
            testCase.verifyTrue(isfinite(res.comparison.improvementPct));
            testCase.verifyGreaterThanOrEqual(res.comparison.improvementAbs, -1e-12);
        end

        function testNoPercentWhenBaselineZero(testCase)
            cfg = fastOptConfig();
            cfg.pGauge = 200; % Too low to lift off -> baseline Hmax == 0
            cfg.p0Abs = [];
            cfgSI = wr.validateConfig(cfg);
            res = wr.optimizeRocket(cfgSI);

            testCase.verifyEqual(res.baseline.Hmax, 0.0, 'AbsTol', 1e-12);
            testCase.verifyTrue(isnan(res.comparison.improvementPct));
            testCase.verifyFalse(isinf(res.comparison.improvementPct));
        end

        function testPlotResultsAndEntryPoint(testCase)
            cfg = fastOptConfig();
            cfg.showPlots = false;
            [res, figs] = runOptimization(cfg);
            cleanupFigs = onCleanup(@() closeAllValidFigs(figs));

            testCase.verifyEqual(string(res.best.sim.status), "success");
            testCase.verifyTrue(isstruct(figs));
            testCase.verifyTrue(isfield(figs, 'overview'));
            testCase.verifyTrue(isgraphics(figs.overview, 'figure'));
            axList = findall(figs.overview, 'Type', 'axes');
            testCase.verifyGreaterThanOrEqual(numel(axList), 6);
        end
    end
end

function closeAllValidFigs(figs)
    if isstruct(figs)
        fNames = fieldnames(figs);
        for k = 1:numel(fNames)
            h = figs.(fNames{k});
            if ~isempty(h) && all(isgraphics(h))
                close(h);
            end
        end
    end
end

function cfg = fastOptConfig()
    cfg = wr.defaultConfig();
    cfg.nControlKnots = 5;
    cfg.nMultiStart = 1;
    cfg.relTol = 1e-5;
    cfg.absTol = 1e-7;
    cfg.maxIter = 15;
end
