classdef TestSimulation < matlab.unittest.TestCase
    %TESTSIMULATION Unit tests for wr.rocketEvents and wr.simulateRocket

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testWaterToAirContinuity(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            f = 0.35;
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            sim = wr.simulateRocket(f, alpha, cfgSI);
            testCase.verifyEqual(string(sim.status), "success");
            testCase.verifyEqual(string(sim.phaseEvents.waterEnd.reason), "waterDepleted");

            % Strict monotonicity of time vector (no duplicate boundary points)
            testCase.verifyTrue(all(diff(sim.t) > 0));

            % Locate Phase W -> Phase A transition index
            idxWEnd = find(sim.phaseId == 1, 1, 'last');
            idxAStart = find(sim.phaseId == 2, 1, 'first');
            testCase.verifyNotEmpty(idxWEnd);
            testCase.verifyNotEmpty(idxAStart);
            testCase.verifyEqual(idxAStart, idxWEnd + 1);

            % Check state continuity across the boundary step
            dtBoundary = sim.t(idxAStart) - sim.t(idxWEnd);
            testCase.verifyLessThan(dtBoundary, 0.05);
            testCase.verifyEqual(sim.Vw(idxAStart), 0.0, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(sim.p(idxWEnd), cfgSI.pAmb);

            % Exact continuity at the stored event state
            yWEnd = sim.phaseEvents.waterEnd.yEnd;
            yAStart = sim.phaseEvents.airStart.yStart;
            testCase.verifyEqual(yAStart(1:4), yWEnd(1:4), 'AbsTol', 1e-12);
            mWEnd = cfgSI.mDry + cfgSI.rhoW * yWEnd(5) + yWEnd(6);
            mAStart = cfgSI.mDry + cfgSI.rhoW * yAStart(5) + yAStart(6);
            testCase.verifyEqual(mAStart, mWEnd, 'AbsTol', 1e-9);
        end

        function testResidualWaterAtPressureCutoff(testCase)
            cfg = wr.defaultConfig();
            cfg.mDry = 0.10;
            cfg.Amax = 4.0e-4;
            cfg.pGauge = 150000;
            cfg.p0Abs = [];
            cfg.fillMax = 0.85;
            cfgSI = wr.validateConfig(cfg);

            f = 0.75; % High fill fraction so air pressure reaches pAmb before water empties
            alpha = 0.9 * ones(1, cfgSI.nControlKnots);

            sim = wr.simulateRocket(f, alpha, cfgSI);
            testCase.verifyEqual(string(sim.status), "success");
            testCase.verifyEqual(string(sim.phaseEvents.waterEnd.reason), "pressureEqualized");
            testCase.verifyEqual(string(sim.phaseEvents.airEnd.reason), "skipped");
            testCase.verifyGreaterThan(sim.phaseEvents.waterEnd.VwRemaining, 0.1 * cfgSI.Vb);
            testCase.verifyEqual(sim.Vw(end), sim.phaseEvents.waterEnd.VwRemaining, 'AbsTol', 1e-12);
        end

        function testApexNotAtStart(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            f = 0.35;
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            sim = wr.simulateRocket(f, alpha, cfgSI);
            testCase.verifyEqual(string(sim.status), "success");
            testCase.verifyGreaterThan(sim.tApex, 0.5);
            testCase.verifyGreaterThan(sim.Hmax, 5.0);
            testCase.verifyEqual(sim.vz(1), 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(sim.vz(end), 0.0, 'AbsTol', 1e-6);
        end

        function testNoLiftoffIsNotSuccess(testCase)
            % Case 1: Initial pressure equal to ambient (bypassing validateConfig)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            cfgSI.p0Abs = cfgSI.pAmb;
            cfgSI.pGauge = 0.0;
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);
            sim1 = wr.simulateRocket(0.35, alpha, cfgSI);
            testCase.verifyNotEqual(string(sim1.status), "success");
            testCase.verifyEqual(string(sim1.status), "noLiftoff");

            % Case 2: Positive gauge pressure but thrust < gravity (cannot lift off)
            cfg2 = wr.defaultConfig();
            cfg2.pGauge = 500; % 0.5 kPa gauge -> tiny thrust
            cfg2.p0Abs = [];
            cfgSI2 = wr.validateConfig(cfg2);
            sim2 = wr.simulateRocket(0.35, alpha, cfgSI2);
            testCase.verifyNotEqual(string(sim2.status), "success");
            testCase.verifyEqual(string(sim2.status), "noLiftoff");
        end

        function testClosedNozzleTimeoutOrFailure(testCase)
            cfg = wr.defaultConfig();
            cfg.Amin = 0.0;
            cfgSI = wr.validateConfig(cfg);
            alphaZero = zeros(1, cfgSI.nControlKnots);

            sim = wr.simulateRocket(0.35, alphaZero, cfgSI);
            testCase.verifyNotEqual(string(sim.status), "success");
            testCase.verifyEqual(string(sim.status), "noLiftoff");
        end

        function testStateNonnegative(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            sim = wr.simulateRocket(0.35, 0.5 * ones(1, cfgSI.nControlKnots), cfgSI);
            testCase.verifyEqual(string(sim.status), "success");
            testCase.verifyGreaterThanOrEqual(sim.Vw, 0.0);
            testCase.verifyGreaterThanOrEqual(sim.ma, 0.0);
            testCase.verifyGreaterThanOrEqual(sim.z, 0.0);
            testCase.verifyGreaterThanOrEqual(sim.p, cfgSI.pAmb - 1e-6);
        end

        function testMaxTimeTimeout(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            cfgSI.maxFlightTime = 0.05; % Too short to reach apex
            sim = wr.simulateRocket(0.35, 0.5 * ones(1, cfgSI.nControlKnots), cfgSI);
            testCase.verifyEqual(string(sim.status), "timeout");
        end
    end
end
