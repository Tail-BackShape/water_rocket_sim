classdef TestSimulation < matlab.unittest.TestCase
    %TESTSIMULATION wr.rocketEvents と wr.simulateRocket の単体テスト

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

            % 時刻ベクトルの単調増加を確認 (境界の重複登録なし)
            testCase.verifyTrue(all(diff(sim.t) > 0));

            % Phase W -> Phase A の境界インデックス特定
            idxWEnd = find(sim.phaseId == 1, 1, 'last');
            idxAStart = find(sim.phaseId == 2, 1, 'first');
            testCase.verifyNotEmpty(idxWEnd);
            testCase.verifyNotEmpty(idxAStart);
            testCase.verifyEqual(idxAStart, idxWEnd + 1);

            dtBoundary = sim.t(idxAStart) - sim.t(idxWEnd);
            testCase.verifyLessThan(dtBoundary, 0.05);
            testCase.verifyEqual(sim.Vw(idxAStart), 0.0, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(sim.p(idxWEnd), cfgSI.pAmb);

            % フェーズ切替イベント状態での位置・速度・質量の連続性確認
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

            % 水が残ったまま内圧が外気圧に達する高充填率条件
            f = 0.75;
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
            % ケース1: 初圧が外気圧と等しい場合
            cfgSI = wr.validateConfig(wr.defaultConfig());
            cfgSI.p0Abs = cfgSI.pAmb;
            cfgSI.pGauge = 0.0;
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);
            sim1 = wr.simulateRocket(0.35, alpha, cfgSI);
            testCase.verifyNotEqual(string(sim1.status), "success");
            testCase.verifyEqual(string(sim1.status), "noLiftoff");

            % ケース2: 差圧が微小で推力が重力を下回る場合
            cfg2 = wr.defaultConfig();
            cfg2.pGauge = 500;
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
            cfgSI.maxFlightTime = 0.05;
            sim = wr.simulateRocket(0.35, 0.5 * ones(1, cfgSI.nControlKnots), cfgSI);
            testCase.verifyEqual(string(sim.status), "timeout");
        end
    end
end
