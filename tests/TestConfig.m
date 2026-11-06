classdef TestConfig < matlab.unittest.TestCase
    %TESTCONFIG wr.defaultConfig と wr.validateConfig の単体テスト

    methods (TestClassSetup)
        function addProjectRootToPath(testCase) %#ok<MANU>
            rootDir = fileparts(fileparts(mfilename('fullpath')));
            addpath(rootDir);
        end
    end

    methods (Test)
        function testDefaultConfigValidates(testCase)
            cfg = wr.defaultConfig();
            cfgSI = wr.validateConfig(cfg);
            testCase.verifyTrue(isstruct(cfgSI));
            requiredFields = { ...
                'Vb', 'mDry', 'D', 'p0Abs', 'theta0', 'T0', 'pAmb', ...
                'rhoAir', 'rhoW', 'Cd', 'gamma', 'Rair', 'g', ...
                'Amin', 'Amax', 'ARateMax', 'tControlEnd', 'nControlKnots', ...
                'fillMin', 'fillMax', 'vEpsilon', 'relTol', 'absTol', ...
                'maxStep', 'maxFlightTime'};
            for i = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(cfgSI, requiredFields{i}), ...
                    sprintf('Missing field: %s', requiredFields{i}));
            end
        end

        function testPressureConversion(testCase)
            cfg = wr.defaultConfig();
            cfg.pAmb = 101325;
            cfg.pGauge = 350000;
            cfg.p0Abs = [];
            cfgSI = wr.validateConfig(cfg);
            testCase.verifyEqual(cfgSI.p0Abs, cfg.pGauge + cfg.pAmb, 'AbsTol', 1e-9);
            testCase.verifyEqual(cfgSI.pGauge, cfg.pGauge, 'AbsTol', 1e-9);
        end

        function testUserUnitConversions(testCase)
            cfg = wr.defaultConfig();
            cfg.Vb = [];
            cfg.Vb_L = 1.5;
            cfg.D = [];
            cfg.D_mm = 90;
            cfg.theta0 = [];
            cfg.theta0_deg = 80;
            cfg.p0Abs = [];
            cfg.pGauge = [];
            cfg.pGauge_kPa = 400;
            cfgSI = wr.validateConfig(cfg);
            testCase.verifyEqual(cfgSI.Vb, 1.5e-3, 'AbsTol', 1e-12);
            testCase.verifyEqual(cfgSI.D, 0.090, 'AbsTol', 1e-12);
            testCase.verifyEqual(cfgSI.theta0, deg2rad(80), 'AbsTol', 1e-12);
            testCase.verifyEqual(cfgSI.p0Abs, 400e3 + cfgSI.pAmb, 'AbsTol', 1e-9);
        end

        function testRejectNearlyFull(testCase)
            cfg = wr.defaultConfig();
            cfg.fillMax = 1.0;
            testCase.verifyError(@() wr.validateConfig(cfg), 'wr:invalidConfig');

            cfg2 = wr.defaultConfig();
            cfg2.fillMin = 0.0;
            testCase.verifyError(@() wr.validateConfig(cfg2), 'wr:invalidConfig');

            cfg3 = wr.defaultConfig();
            cfg3.fillMin = 0.6;
            cfg3.fillMax = 0.4;
            testCase.verifyError(@() wr.validateConfig(cfg3), 'wr:invalidConfig');
        end

        function testRejectBadNozzleBounds(testCase)
            cfg = wr.defaultConfig();
            cfg.Amax = cfg.Amin;
            testCase.verifyError(@() wr.validateConfig(cfg), 'wr:invalidConfig');

            cfg2 = wr.defaultConfig();
            cfg2.Amin = -1e-6;
            testCase.verifyError(@() wr.validateConfig(cfg2), 'wr:invalidConfig');

            cfg3 = wr.defaultConfig();
            cfg3.ARateMax = 0;
            testCase.verifyError(@() wr.validateConfig(cfg3), 'wr:invalidConfig');
        end

        function testMissingDryMass(testCase)
            cfg = wr.defaultConfig();
            cfg.mDry = [];
            testCase.verifyError(@() wr.validateConfig(cfg), 'wr:invalidConfig');

            cfg2 = wr.defaultConfig();
            cfg2.mDry = -0.1;
            testCase.verifyError(@() wr.validateConfig(cfg2), 'wr:invalidConfig');
        end

        function testRejectConflictingPressures(testCase)
            cfg = wr.defaultConfig();
            cfg.pAmb = 101325;
            cfg.pGauge = 300000;
            cfg.p0Abs = 500000;
            testCase.verifyError(@() wr.validateConfig(cfg), 'wr:invalidConfig');
        end
    end
end
