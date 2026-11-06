classdef TestDynamics < matlab.unittest.TestCase
    %TESTDYNAMICS Unit tests for wr.rocketRHS (2D point-mass equations of motion)

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testZeroSpeedUsesLaunchAngle(testCase)
            cfg = wr.defaultConfig();
            cfg.theta0 = deg2rad(60);
            cfg.Cd = 0.0;
            cfgSI = wr.validateConfig(cfg);

            Vw0 = 0.35 * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            y0 = [0; 0; 0; 0; Vw0; ma0];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            prop = wr.waterPropulsion(0.0, y0, alpha, cfgSI);
            mTot = cfgSI.mDry + cfgSI.rhoW * Vw0 + ma0;

            dy = wr.rocketRHS(0.0, y0, "water", alpha, cfgSI);
            testCase.verifyEqual(size(dy), [6, 1]);
            testCase.verifyEqual(dy(1:2), [0; 0], 'AbsTol', 1e-15);
            testCase.verifyEqual(dy(3), (prop.thrust * cos(cfgSI.theta0)) / mTot, 'RelTol', 1e-12);
            testCase.verifyEqual(dy(4), (prop.thrust * sin(cfgSI.theta0)) / mTot - cfgSI.g, 'RelTol', 1e-12);
        end

        function testVelocityAlignmentAfterThreshold(testCase)
            cfg = wr.defaultConfig();
            cfg.theta0 = deg2rad(85);
            cfg.Cd = 0.0;
            cfgSI = wr.validateConfig(cfg);

            Vw0 = 0.35 * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            % Velocity at 45 degrees with magnitude > vEpsilon
            vMag = 10.0 * cfgSI.vEpsilon;
            vx = vMag / sqrt(2);
            vz = vMag / sqrt(2);
            y = [1; 2; vx; vz; Vw0; ma0];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            prop = wr.waterPropulsion(0.05, y, alpha, cfgSI);
            mTot = cfgSI.mDry + cfgSI.rhoW * Vw0 + ma0;

            dy = wr.rocketRHS(0.05, y, "water", alpha, cfgSI);
            testCase.verifyEqual(dy(3), (prop.thrust / sqrt(2)) / mTot, 'RelTol', 1e-12);
            testCase.verifyEqual(dy(4), (prop.thrust / sqrt(2)) / mTot - cfgSI.g, 'RelTol', 1e-12);
        end

        function testDragOpposesVelocity(testCase)
            cfg = wr.defaultConfig();
            cfg.Cd = 0.50;
            cfgSI = wr.validateConfig(cfg);

            ma = 0.002;
            vx = 12.0;
            vz = 16.0;
            y = [5; 10; vx; vz; 0; ma];
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            dy = wr.rocketRHS(1.0, y, "coast", alpha, cfgSI);
            mTot = cfgSI.mDry + ma;
            vNorm = hypot(vx, vz);
            S = pi * cfgSI.D^2 / 4;
            dragFactor = 0.5 * cfgSI.rhoAir * cfgSI.Cd * S * vNorm / mTot;

            testCase.verifyEqual(dy(3), -dragFactor * vx, 'RelTol', 1e-12);
            testCase.verifyEqual(dy(4), -dragFactor * vz - cfgSI.g, 'RelTol', 1e-12);
            testCase.verifyLessThan(dy(3), 0.0);
        end

        function testVerticalSymmetry(testCase)
            cfg = wr.defaultConfig();
            cfg.theta0 = pi / 2;
            cfgSI = wr.validateConfig(cfg);

            Vw0 = 0.35 * cfgSI.Vb;
            Va0 = cfgSI.Vb - Vw0;
            ma0 = cfgSI.p0Abs * Va0 / (cfgSI.Rair * cfgSI.T0);
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            yZero = [0; 0; 0; 0; Vw0; ma0];
            dyZero = wr.rocketRHS(0.0, yZero, "water", alpha, cfgSI);
            testCase.verifyEqual(dyZero(1), 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dyZero(3), 0.0, 'AbsTol', 1e-14);

            yMoving = [0; 5; 0; 15.0; 0.5 * Vw0; ma0];
            dyMoving = wr.rocketRHS(0.05, yMoving, "water", alpha, cfgSI);
            testCase.verifyEqual(dyMoving(1), 0.0, 'AbsTol', 1e-15);
            testCase.verifyEqual(dyMoving(3), 0.0, 'AbsTol', 1e-14);
        end

        function testMassUsesRemainingPropellant(testCase)
            cfg = wr.defaultConfig();
            cfg.Cd = 0.6;
            cfgSI = wr.validateConfig(cfg);
            alpha = 0.5 * ones(1, cfgSI.nControlKnots);

            yLight = [0; 10; 10; 20; 0.0; 0.002];
            yHeavy = [0; 10; 10; 20; 0.3 * cfgSI.Vb; 0.002];

            dyLight = wr.rocketRHS(0.5, yLight, "coast", alpha, cfgSI);
            dyHeavy = wr.rocketRHS(0.5, yHeavy, "coast", alpha, cfgSI);

            % Drag deceleration magnitude is smaller for heavier remaining mass
            testCase.verifyLessThan(abs(dyHeavy(3)), abs(dyLight(3)));
        end
    end
end
