classdef TestNozzle < matlab.unittest.TestCase
    %TESTNOZZLE wr.openingAtTime とノズル開度補間・変化速度制約の単体テスト

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testConstantKnots(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            alpha0 = 0.4;
            alpha = alpha0 * ones(1, cfgSI.nControlKnots);
            tQuery = linspace(-0.05, cfgSI.tControlEnd + 0.2, 50);
            [A, dEq] = wr.openingAtTime(tQuery, alpha, cfgSI);

            expectedA = cfgSI.Amin + (cfgSI.Amax - cfgSI.Amin) * alpha0;
            expectedDEq = sqrt(4 * expectedA / pi);
            testCase.verifyEqual(A, expectedA * ones(size(tQuery)), 'AbsTol', 1e-15);
            testCase.verifyEqual(dEq, expectedDEq * ones(size(tQuery)), 'AbsTol', 1e-15);
        end

        function testLinearMidpoint(testCase)
            cfg = wr.defaultConfig();
            cfg.nControlKnots = 5;
            cfg.tControlEnd = 0.40;
            cfgSI = wr.validateConfig(cfg);

            alpha = [0.0, 0.2, 0.8, 0.4, 1.0];
            tKnots = linspace(0, cfgSI.tControlEnd, cfgSI.nControlKnots);
            [AKnots, ~] = wr.openingAtTime(tKnots, alpha, cfgSI);

            tMid = 0.5 * (tKnots(1:end-1) + tKnots(2:end));
            [AMid, ~] = wr.openingAtTime(tMid, alpha, cfgSI);
            expectedMid = 0.5 * (AKnots(1:end-1) + AKnots(2:end));
            testCase.verifyEqual(AMid, expectedMid, 'AbsTol', 1e-15);
        end

        function testEndpointHold(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            alpha = linspace(0.15, 0.85, cfgSI.nControlKnots);

            [AStart, ~] = wr.openingAtTime(0.0, alpha, cfgSI);
            [ABefore, ~] = wr.openingAtTime([-1.0, -0.01, 0.0], alpha, cfgSI);
            testCase.verifyEqual(ABefore, AStart * ones(1, 3), 'AbsTol', 1e-15);

            [AEnd, dEqEnd] = wr.openingAtTime(cfgSI.tControlEnd, alpha, cfgSI);
            [AAfter, dEqAfter] = wr.openingAtTime(cfgSI.tControlEnd + [0.0, 0.05, 1.0, 10.0], alpha, cfgSI);
            testCase.verifyEqual(AAfter, AEnd * ones(1, 4), 'AbsTol', 1e-15);
            testCase.verifyEqual(dEqAfter, dEqEnd * ones(1, 4), 'AbsTol', 1e-15);
        end

        function testSlewBound(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            tKnots = linspace(0, cfgSI.tControlEnd, cfgSI.nControlKnots);
            dt = tKnots(2) - tKnots(1);
            dAlphaMax = cfgSI.ARateMax * dt / (cfgSI.Amax - cfgSI.Amin);

            % 最大変化速度ちょうどで推移する実行可能スケジュールを生成
            alpha = zeros(1, cfgSI.nControlKnots);
            alpha(1) = 0.1;
            for k = 2:cfgSI.nControlKnots
                if mod(k, 2) == 0
                    alpha(k) = min(1.0, alpha(k-1) + dAlphaMax);
                else
                    alpha(k) = max(0.0, alpha(k-1) - dAlphaMax);
                end
            end

            [AKnots, ~] = wr.openingAtTime(tKnots, alpha, cfgSI);
            knotRates = abs(diff(AKnots)) ./ diff(tKnots);
            testCase.verifyLessThanOrEqual(knotRates, cfgSI.ARateMax + 1e-12);

            % 細密な時間グリッドでも変化速度上限を満たすことを確認
            tFine = linspace(0, cfgSI.tControlEnd * 1.5, 300);
            [AFine, ~] = wr.openingAtTime(tFine, alpha, cfgSI);
            fineRates = abs(diff(AFine)) ./ diff(tFine);
            testCase.verifyLessThanOrEqual(fineRates, cfgSI.ARateMax + 1e-12);
        end

        function testNoOvershoot(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            alpha = [0, 1, 0, 1, 0.5, 1];
            alpha = alpha(1:cfgSI.nControlKnots);
            tFine = linspace(-0.1, cfgSI.tControlEnd + 0.5, 500);
            [A, dEq] = wr.openingAtTime(tFine, alpha, cfgSI);

            testCase.verifyGreaterThanOrEqual(A, cfgSI.Amin - 1e-15);
            testCase.verifyLessThanOrEqual(A, cfgSI.Amax + 1e-15);
            testCase.verifyEqual(dEq, sqrt(4 * A / pi), 'AbsTol', 1e-15);
        end
    end
end
