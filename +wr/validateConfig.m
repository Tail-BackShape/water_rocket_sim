function cfgSI = validateConfig(cfg)
%VALIDATECONFIG 設定構造体を検証し、ユーザー表示単位をSI単位へ変換する
%   不正な設定値がある場合は 'wr:invalidConfig' エラーを送出する。

    if nargin < 1 || ~isstruct(cfg) || numel(cfg) ~= 1
        error('wr:invalidConfig', 'Configuration must be a scalar struct.');
    end

    cfgSI = cfg;

    % --- 1. SIフィールドが空の場合にユーザー表示単位から換算 ---
    if (~isfield(cfgSI, 'Vb') || isempty(cfgSI.Vb)) && isfield(cfgSI, 'Vb_L') && ~isempty(cfgSI.Vb_L)
        validateScalarPositive(cfgSI.Vb_L, 'Vb_L');
        cfgSI.Vb = cfgSI.Vb_L * 1e-3;
    end

    if (~isfield(cfgSI, 'D') || isempty(cfgSI.D)) && isfield(cfgSI, 'D_mm') && ~isempty(cfgSI.D_mm)
        validateScalarPositive(cfgSI.D_mm, 'D_mm');
        cfgSI.D = cfgSI.D_mm * 1e-3;
    end

    if (~isfield(cfgSI, 'theta0') || isempty(cfgSI.theta0)) && isfield(cfgSI, 'theta0_deg') && ~isempty(cfgSI.theta0_deg)
        validateScalarReal(cfgSI.theta0_deg, 'theta0_deg');
        cfgSI.theta0 = deg2rad(cfgSI.theta0_deg);
    end

    if (~isfield(cfgSI, 'Amin') || isempty(cfgSI.Amin)) && isfield(cfgSI, 'dMin_mm') && ~isempty(cfgSI.dMin_mm)
        validateScalarNonnegative(cfgSI.dMin_mm, 'dMin_mm');
        cfgSI.Amin = pi * (cfgSI.dMin_mm * 1e-3 / 2)^2;
    end
    if (~isfield(cfgSI, 'Amax') || isempty(cfgSI.Amax)) && isfield(cfgSI, 'dMax_mm') && ~isempty(cfgSI.dMax_mm)
        validateScalarPositive(cfgSI.dMax_mm, 'dMax_mm');
        cfgSI.Amax = pi * (cfgSI.dMax_mm * 1e-3 / 2)^2;
    end

    % --- 2. 外気圧と初期タンク圧力の検証・換算 ---
    requireField(cfgSI, 'pAmb');
    validateScalarPositive(cfgSI.pAmb, 'pAmb');

    hasP0Abs = isfield(cfgSI, 'p0Abs') && ~isempty(cfgSI.p0Abs);
    hasPGauge = isfield(cfgSI, 'pGauge') && ~isempty(cfgSI.pGauge);
    hasPGaugeKPa = isfield(cfgSI, 'pGauge_kPa') && ~isempty(cfgSI.pGauge_kPa);

    if hasP0Abs && hasPGauge
        validateScalarReal(cfgSI.p0Abs, 'p0Abs');
        validateScalarReal(cfgSI.pGauge, 'pGauge');
        if abs(cfgSI.p0Abs - (cfgSI.pGauge + cfgSI.pAmb)) > 1e-6
            error('wr:invalidConfig', ...
                'Conflicting p0Abs (%.3f Pa) and pGauge (%.3f Pa) with pAmb (%.3f Pa).', ...
                cfgSI.p0Abs, cfgSI.pGauge, cfgSI.pAmb);
        end
    elseif hasP0Abs
        validateScalarReal(cfgSI.p0Abs, 'p0Abs');
        cfgSI.pGauge = cfgSI.p0Abs - cfgSI.pAmb;
    elseif hasPGauge
        validateScalarReal(cfgSI.pGauge, 'pGauge');
        cfgSI.p0Abs = cfgSI.pGauge + cfgSI.pAmb;
    elseif hasPGaugeKPa
        validateScalarReal(cfgSI.pGauge_kPa, 'pGauge_kPa');
        cfgSI.pGauge = cfgSI.pGauge_kPa * 1e3;
        cfgSI.p0Abs = cfgSI.pGauge + cfgSI.pAmb;
    else
        error('wr:invalidConfig', 'Initial pressure must be specified via p0Abs, pGauge, or pGauge_kPa.');
    end

    if cfgSI.p0Abs <= cfgSI.pAmb
        error('wr:invalidConfig', ...
            'Initial absolute pressure p0Abs (%.3f Pa) must exceed ambient pressure pAmb (%.3f Pa).', ...
            cfgSI.p0Abs, cfgSI.pAmb);
    end

    % --- 3. 機体・環境物性パラメータの検証 ---
    requireField(cfgSI, 'Vb');
    validateScalarPositive(cfgSI.Vb, 'Vb');

    requireField(cfgSI, 'mDry');
    validateScalarPositive(cfgSI.mDry, 'mDry');

    requireField(cfgSI, 'D');
    validateScalarPositive(cfgSI.D, 'D');

    requireField(cfgSI, 'theta0');
    validateScalarReal(cfgSI.theta0, 'theta0');
    if cfgSI.theta0 <= 0 || cfgSI.theta0 > pi / 2
        error('wr:invalidConfig', 'theta0 must be in (0, pi/2] radians.');
    end

    requireField(cfgSI, 'T0');
    validateScalarPositive(cfgSI.T0, 'T0');

    requireField(cfgSI, 'rhoAir');
    validateScalarNonnegative(cfgSI.rhoAir, 'rhoAir');

    requireField(cfgSI, 'rhoW');
    validateScalarPositive(cfgSI.rhoW, 'rhoW');

    requireField(cfgSI, 'Cd');
    validateScalarNonnegative(cfgSI.Cd, 'Cd');

    requireField(cfgSI, 'gamma');
    validateScalarReal(cfgSI.gamma, 'gamma');
    if cfgSI.gamma <= 1.0
        error('wr:invalidConfig', 'gamma must be strictly greater than 1.');
    end

    requireField(cfgSI, 'Rair');
    validateScalarPositive(cfgSI.Rair, 'Rair');

    requireField(cfgSI, 'g');
    validateScalarPositive(cfgSI.g, 'g');

    % --- 4. ノズル・制御パラメータの検証 ---
    requireField(cfgSI, 'Amin');
    validateScalarNonnegative(cfgSI.Amin, 'Amin');

    requireField(cfgSI, 'Amax');
    validateScalarPositive(cfgSI.Amax, 'Amax');

    if cfgSI.Amax <= cfgSI.Amin
        error('wr:invalidConfig', 'Amax must be strictly greater than Amin.');
    end

    requireField(cfgSI, 'ARateMax');
    validateScalarPositive(cfgSI.ARateMax, 'ARateMax');

    requireField(cfgSI, 'tControlEnd');
    validateScalarPositive(cfgSI.tControlEnd, 'tControlEnd');

    requireField(cfgSI, 'nControlKnots');
    validateScalarPositive(cfgSI.nControlKnots, 'nControlKnots');
    if floor(cfgSI.nControlKnots) ~= cfgSI.nControlKnots || cfgSI.nControlKnots < 2
        error('wr:invalidConfig', 'nControlKnots must be an integer >= 2.');
    end

    requireField(cfgSI, 'fillMin');
    requireField(cfgSI, 'fillMax');
    validateScalarReal(cfgSI.fillMin, 'fillMin');
    validateScalarReal(cfgSI.fillMax, 'fillMax');
    if ~(cfgSI.fillMin > 0 && cfgSI.fillMin < cfgSI.fillMax && cfgSI.fillMax < 1)
        error('wr:invalidConfig', 'Water fill fraction bounds must satisfy 0 < fillMin < fillMax < 1.');
    end

    % --- 5. 数値積分ソルバ設定の検証 ---
    requireField(cfgSI, 'vEpsilon');
    validateScalarPositive(cfgSI.vEpsilon, 'vEpsilon');

    requireField(cfgSI, 'relTol');
    validateScalarPositive(cfgSI.relTol, 'relTol');

    requireField(cfgSI, 'absTol');
    if ~isnumeric(cfgSI.absTol) || isempty(cfgSI.absTol) || any(~isfinite(cfgSI.absTol(:))) || any(cfgSI.absTol(:) <= 0)
        error('wr:invalidConfig', 'absTol must be positive finite numeric value(s).');
    end
    if ~(isscalar(cfgSI.absTol) || numel(cfgSI.absTol) == 6)
        error('wr:invalidConfig', 'absTol must be a scalar or a 6-element vector.');
    end

    requireField(cfgSI, 'maxStep');
    validateScalarPositive(cfgSI.maxStep, 'maxStep');

    requireField(cfgSI, 'maxFlightTime');
    validateScalarPositive(cfgSI.maxFlightTime, 'maxFlightTime');
    if cfgSI.maxFlightTime <= cfgSI.tControlEnd
        error('wr:invalidConfig', 'maxFlightTime must be greater than tControlEnd.');
    end

    % 補助量の事前計算
    cfgSI.tKnots = linspace(0, cfgSI.tControlEnd, cfgSI.nControlKnots);
    cfgSI.bodyArea = pi * (cfgSI.D^2) / 4;
end

function requireField(s, name)
    if ~isfield(s, name) || isempty(s.(name))
        error('wr:invalidConfig', 'Required field "%s" is missing or empty.', name);
    end
end

function validateScalarReal(val, name)
    if ~isnumeric(val) || ~isscalar(val) || ~isreal(val) || ~isfinite(val)
        error('wr:invalidConfig', 'Field "%s" must be a finite real scalar.', name);
    end
end

function validateScalarPositive(val, name)
    validateScalarReal(val, name);
    if val <= 0
        error('wr:invalidConfig', 'Field "%s" must be strictly positive.', name);
    end
end

function validateScalarNonnegative(val, name)
    validateScalarReal(val, name);
    if val < 0
        error('wr:invalidConfig', 'Field "%s" must be non-negative.', name);
    end
end
