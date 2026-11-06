classdef TestWaterPropulsion < matlab.unittest.TestCase
    %TESTWATERPROPULSION wr.waterPropulsion (Phase W) の単体テスト

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testInitialPressure(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            f = 0.35;
            Vw0 = f * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            y0 = [0; 0; 0; 0; Vw0; ma0];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            prop = wr.waterPropulsion(0.0, y0, alpha, cfgSI);
            testCase.verifyEqual(prop.p, cfgSI.p0Abs, 'RelTol', 1e-12);
            testCase.verifyEqual(prop.T, cfgSI.T0, 'RelTol', 1e-12);
            testCase.verifyEqual(prop.dMa, 0.0, 'AbsTol', 1e-15);
            testCase.verifyLessThan(prop.dVw, 0.0);
            testCase.verifyGreaterThan(prop.thrust, 0.0);
        end

        function testPressureDrops(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            f = 0.40;
            Vw0 = f * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            y0 = [0; 0; 0; 0; Vw0; ma0];
            yLater = [0; 1; 0; 10; 0.5 * Vw0; ma0];

            prop0 = wr.waterPropulsion(0.0, y0, alpha, cfgSI);
            propLater = wr.waterPropulsion(0.05, yLater, alpha, cfgSI);

            testCase.verifyLessThan(propLater.p, prop0.p);
            testCase.verifyLessThan(propLater.T, prop0.T);
            expectedP = cfgSI.p0Abs * (Va0 / (cfgSI.Vb - 0.5 * Vw0))^cfgSI.gamma;
            testCase.verifyEqual(propLater.p, expectedP, 'RelTol', 1e-12);
        end

        function testAreaScaling(testCase)
            cfg = wr.defaultConfig();
            cfg.Amin = 1.0e-5;
            cfg.Amax = 5.0e-5;
            cfgSI = wr.validateConfig(cfg);

            Vw0 = 0.35 * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            y = [0; 0; 0; 0; Vw0; ma0];

            % alpha1: A = 2.0e-5, alpha2: A = 4.0e-5 (面積2倍)
            alpha1 = 0.25 * ones(1, cfgSI.nControlKnots);
            alpha2 = 0.75 * ones(1, cfgSI.nControlKnots);

            prop1 = wr.waterPropulsion(0.0, y, alpha1, cfgSI);
            prop2 = wr.waterPropulsion(0.0, y, alpha2, cfgSI);

            testCase.verifyEqual(prop2.dVw, 2.0 * prop1.dVw, 'RelTol', 1e-12);
            testCase.verifyEqual(prop2.thrust, 2.0 * prop1.thrust, 'RelTol', 1e-12);
            testCase.verifyEqual(prop2.mDotOut, 2.0 * prop1.mDotOut, 'RelTol', 1e-12);
        end

        function testNoReverseFlow(testCase)
            cfg = wr.defaultConfig();
            cfg.pGauge = 20000;
            cfg.p0Abs = [];
            cfgSI = wr.validateConfig(cfg);

            % 高い初期水率から膨張させ、水が抜ける前に外気圧以下となる状態を作る
            Vw0 = 0.80 * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);

            yExpanded = [0; 5; 0; 10; 0.10 * cfgSI.Vb; ma0];
            alpha = 0.8 * ones(1, cfgSI.nControlKnots);

            prop = wr.waterPropulsion(0.10, yExpanded, alpha, cfgSI);
            testCase.verifyLessThanOrEqual(prop.p, cfgSI.pAmb);
            testCase.verifyEqual(prop.dVw, 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(prop.thrust, 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(prop.mDotOut, 0.0, 'AbsTol', 1e-15);
        end
    end
end
