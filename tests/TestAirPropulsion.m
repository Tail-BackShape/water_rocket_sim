classdef TestAirPropulsion < matlab.unittest.TestCase
    %TESTAIRPROPULSION Unit tests for wr.airPropulsion (Phase A)

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testChokedFlowPositive(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            % Set p1 well above critical pressure: pAmb / p1 < rCrit (~0.5283)
            phase0 = struct('p1', 3.0 * cfgSI.pAmb, 'T1', 260.0, 'ma1', 0.005);
            y = [0; 10; 0; 25; 0; phase0.ma1];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            prop = wr.airPropulsion(0.15, y, alpha, cfgSI, phase0);
            testCase.verifyTrue(prop.isChoked);
            testCase.verifyGreaterThan(prop.mDotOut, 0.0);
            testCase.verifyEqual(prop.dMa, -prop.mDotOut, 'AbsTol', 1e-15);
            testCase.verifyEqual(prop.dVw, 0.0, 'AbsTol', 1e-15);
            testCase.verifyGreaterThan(prop.thrust, 0.0);
            testCase.verifyGreaterThan(prop.pe, cfgSI.pAmb);
        end

        function testUnchokedFlowPositive(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            % Set p1 moderately above ambient so rCrit < pAmb/p1 < 1
            phase0 = struct('p1', 1.3 * cfgSI.pAmb, 'T1', 250.0, 'ma1', 0.003);
            y = [0; 12; 0; 20; 0; phase0.ma1];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            prop = wr.airPropulsion(0.20, y, alpha, cfgSI, phase0);
            testCase.verifyFalse(prop.isChoked);
            testCase.verifyGreaterThan(prop.mDotOut, 0.0);
            testCase.verifyEqual(prop.dMa, -prop.mDotOut, 'AbsTol', 1e-15);
            testCase.verifyEqual(prop.dVw, 0.0, 'AbsTol', 1e-15);
            testCase.verifyGreaterThan(prop.thrust, 0.0);
            testCase.verifyEqual(prop.pe, cfgSI.pAmb, 'AbsTol', 1e-12);
        end

        function testNoFlowAtAmbient(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            phase0 = struct('p1', 1.5 * cfgSI.pAmb, 'T1', 260.0, 'ma1', 0.004);
            % Choose ma such that p == pAmb
            maAmb = phase0.ma1 * (cfgSI.pAmb / phase0.p1)^(1.0 / cfgSI.gamma);
            yAmb = [0; 15; 0; 15; 0; maAmb];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            propAmb = wr.airPropulsion(0.25, yAmb, alpha, cfgSI, phase0);
            testCase.verifyEqual(propAmb.mDotOut, 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(propAmb.dMa, 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(propAmb.thrust, 0.0, 'AbsTol', 1e-15);

            % Also verify slightly below ambient produces zero reverse flow
            ySub = [0; 15; 0; 15; 0; 0.95 * maAmb];
            propSub = wr.airPropulsion(0.25, ySub, alpha, cfgSI, phase0);
            testCase.verifyEqual(propSub.mDotOut, 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(propSub.thrust, 0.0, 'AbsTol', 1e-15);
        end

        function testPressureDecreasesWithGasMass(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            phase0 = struct('p1', 2.5 * cfgSI.pAmb, 'T1', 270.0, 'ma1', 0.005);
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            y1 = [0; 10; 0; 20; 0; phase0.ma1];
            y2 = [0; 11; 0; 21; 0; 0.85 * phase0.ma1];

            prop1 = wr.airPropulsion(0.15, y1, alpha, cfgSI, phase0);
            prop2 = wr.airPropulsion(0.18, y2, alpha, cfgSI, phase0);

            testCase.verifyLessThan(prop2.p, prop1.p);
            testCase.verifyLessThan(prop2.T, prop1.T);
            expectedP2 = phase0.p1 * (0.85)^cfgSI.gamma;
            testCase.verifyEqual(prop2.p, expectedP2, 'RelTol', 1e-12);
        end

        function testChokeBoundaryContinuous(testCase)
            cfgSI = wr.validateConfig(wr.defaultConfig());
            g = cfgSI.gamma;
            rCrit = (2.0 / (g + 1.0))^(g / (g - 1.0));
            pCrit = cfgSI.pAmb / rCrit;

            phase0 = struct('p1', pCrit, 'T1', 265.0, 'ma1', 0.004);
            alpha = 0.6 * ones(1, cfgSI.nControlKnots);

            epsMass = 1e-7;
            yChokedSide = [0; 10; 0; 20; 0; phase0.ma1 * (1.0 + epsMass)];
            yUnchokedSide = [0; 10; 0; 20; 0; phase0.ma1 * (1.0 - epsMass)];

            propChoked = wr.airPropulsion(0.15, yChokedSide, alpha, cfgSI, phase0);
            propUnchoked = wr.airPropulsion(0.15, yUnchokedSide, alpha, cfgSI, phase0);

            testCase.verifyTrue(propChoked.isChoked);
            testCase.verifyFalse(propUnchoked.isChoked);
            testCase.verifyTrue(isfinite(propChoked.mDotOut) && isfinite(propUnchoked.mDotOut));
            testCase.verifyTrue(isfinite(propChoked.thrust) && isfinite(propUnchoked.thrust));
            testCase.verifyEqual(propChoked.mDotOut, propUnchoked.mDotOut, 'RelTol', 1e-5);
            testCase.verifyEqual(propChoked.thrust, propUnchoked.thrust, 'RelTol', 1e-5);
        end
    end
end
