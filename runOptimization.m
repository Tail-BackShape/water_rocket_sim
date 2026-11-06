function [result, figs] = runOptimization(cfg)
%RUNOPTIMIZATION 設定検証・水ロケット高度最適化・結果描画を実行する
%   [result, figs] = runOptimization() は本ファイル内の設定値を用いて最適化を実行する。
%   [result, figs] = runOptimization(cfg) は指定された構造体 cfg のフィールドで設定を上書きする。

    % =========================================================================
    %  定数・係数・機体パラメータ設定
    % =========================================================================
    userCfg = wr.defaultConfig();
    userCfg.isDemo = true;                    % デモ設定フラグ (実機値入力時は false)

    % --- 1. 機体諸元 ---
    userCfg.Vb     = 1.5 * 1e-3;              % タンク内容積 [m^3] (1.5 L)
    userCfg.mDry   = 0.250;                   % 乾燥質量 (ノズル機構・電装含む) [kg]
    userCfg.D      = 90.0 * 1e-3;             % 胴体最大直径 [m] (90 mm)

    % --- 2. 発射条件・初期内部状態 ---
    userCfg.pAmb   = 101325.0;                % 外気絶対圧 [Pa]
    userCfg.pGauge = 400.0 * 1e3;             % 初期タンクゲージ圧 [Pa] (400 kPaG)
    userCfg.p0Abs  = [];                      % 初期タンク絶対圧 [Pa] (空なら pAmb + pGauge)
    userCfg.theta0 = deg2rad(85.0);           % 発射角 [rad] (水平=0, 鉛直=90 deg)
    userCfg.T0     = 293.15;                  % 初期タンク空気温度 [K] (20 degC)

    % --- 3. 物理定数・環境・空力係数 ---
    userCfg.Cd     = 0.45;                    % 機体の抗力係数 [-]
    userCfg.rhoAir = 1.204;                   % 外気密度 [kg/m^3]
    userCfg.rhoW   = 998.2;                   % 水の密度 [kg/m^3]
    userCfg.gamma  = 1.4;                     % 空気の比熱比 [-]
    userCfg.Rair   = 287.05;                  % 空気の比気体定数 [J/(kg*K)]
    userCfg.g      = 9.80665;                 % 重力加速度 [m/s^2]

    % --- 4. 可変ノズル機構パラメータ ---
    dMin_m           = 5.0 * 1e-3;            % 最小等価ノズル直径 [m]
    dMax_m           = 15.0 * 1e-3;           % 最大等価ノズル直径 [m]
    userCfg.Amin     = pi * (dMin_m / 2)^2;   % 最小有効開口面積 [m^2]
    userCfg.Amax     = pi * (dMax_m / 2)^2;   % 最大有効開口面積 [m^2]
    userCfg.ARateMax = 1.2e-3;                % 有効開口面積の最大変化速度 [m^2/s]

    % --- 5. 最適制御スケジュール設定 ---
    userCfg.tControlEnd   = 0.30;             % 制御点の末尾時刻 [s]
    userCfg.nControlKnots = 6;                % 制御ノット点数 [-]
    userCfg.fillMin       = 0.10;             % 初期水充填率 Vw0/Vb の下限 [-]
    userCfg.fillMax       = 0.65;             % 初期水充填率 Vw0/Vb の上限 [-]

    % --- 6. 数値積分・最適化ソルバ設定 ---
    userCfg.vEpsilon      = 0.20;             % 推力方向を速度方向へ一致させる速度閾値 [m/s]
    userCfg.relTol        = 1e-6;             % ODE 相対許容誤差 [-]
    userCfg.absTol        = 1e-8;             % ODE 絶対許容誤差 [-]
    userCfg.maxStep       = 0.01;             % ODE 最大ステップ幅 [s]
    userCfg.maxFlightTime = 15.0;             % 最大飛行積分時間 [s]
    userCfg.randomSeed    = 42;               % 再現用乱数シード
    userCfg.nMultiStart   = 3;                % マルチスタート初期値数 [-]
    userCfg.maxIter       = 40;               % fmincon 最大反復回数 [-]
    userCfg.useGA         = false;            % Global Optimization Toolbox (ga) 併用フラグ
    userCfg.showPlots     = true;             % 結果グラフ表示フラグ
    % =========================================================================

    % 引数 cfg が渡された場合は該当フィールドを上書き
    if nargin >= 1 && ~isempty(cfg)
        if ~isstruct(cfg) || numel(cfg) ~= 1
            error('wr:invalidConfig', 'Input cfg must be a scalar struct.');
        end
        overrideFields = fieldnames(cfg);
        for i = 1:numel(overrideFields)
            fn = overrideFields{i};
            userCfg.(fn) = cfg.(fn);
        end
    end

    cfgSI = wr.validateConfig(userCfg);
    result = wr.optimizeRocket(cfgSI);
    figs = wr.plotResults(result, cfgSI);

    % --- アニメーション表示 (オン/オフは下記のコメントアウトで切り替え) ---
    figs.animation = wr.animateResults(result, cfgSI);

    printSummaryReport(result, cfgSI);
end

function printSummaryReport(result, cfgSI)
    fprintf('\n====================================================================\n');
    fprintf('  Water Rocket Variable-Nozzle Altitude Optimization Summary\n');
    fprintf('====================================================================\n');
    if isfield(cfgSI, 'isDemo') && cfgSI.isDemo
        fprintf('  [NOTE] Running with DEMO template parameters (not hardware-calibrated).\n');
    end
    fprintf('  Tank volume Vb        : %.3f L (%.6f m^3)\n', cfgSI.Vb * 1e3, cfgSI.Vb);
    fprintf('  Dry mass mDry         : %.3f kg\n', cfgSI.mDry);
    fprintf('  Body diameter D       : %.1f mm (%.4f m)\n', cfgSI.D * 1e3, cfgSI.D);
    fprintf('  Initial pressure      : %.1f kPaG (%.1f kPa abs)\n', ...
        cfgSI.pGauge * 1e-3, cfgSI.p0Abs * 1e-3);
    fprintf('  Launch angle theta0   : %.1f deg (%.4f rad)\n', ...
        rad2deg(cfgSI.theta0), cfgSI.theta0);
    fprintf('  Drag coeff Cd         : %.3f [-]\n', cfgSI.Cd);
    fprintf('  Nozzle area bounds    : [%.3e, %.3e] m^2 (dEq: [%.2f, %.2f] mm)\n', ...
        cfgSI.Amin, cfgSI.Amax, ...
        sqrt(4 * cfgSI.Amin / pi) * 1e3, sqrt(4 * cfgSI.Amax / pi) * 1e3);
    fprintf('  Max area slew rate    : %.3e m^2/s\n', cfgSI.ARateMax);
    fprintf('  Control knots / end   : %d knots over [0, %.3f] s\n', ...
        cfgSI.nControlKnots, cfgSI.tControlEnd);
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Baseline (best const) : Hmax = %.3f m (f = %.3f, Vw0 = %.3f L, alpha = %.3f)\n', ...
        result.baseline.Hmax, result.baseline.f, result.baseline.Vw0_L, result.baseline.alpha(1));
    fprintf('  Optimal (variable)    : Hmax = %.3f m (f* = %.3f, Vw0* = %.3f L, tApex = %.3f s)\n', ...
        result.best.Hmax, result.best.f, result.best.Vw0_L, result.best.tApex);
    if isfinite(result.comparison.improvementPct)
        fprintf('  Altitude improvement  : +%.3f m (+%.2f %%)\n', ...
            result.comparison.improvementAbs, result.comparison.improvementPct);
    else
        fprintf('  Altitude improvement  : %.3f m (percentage N/A: non-positive baseline)\n', ...
            result.comparison.improvementAbs);
    end
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Optimal Knot Schedule :\n');
    for k = 1:numel(result.best.tKnots)
        fprintf('    k=%d: t=%.3f s | alpha=%.4f | A=%.4e m^2 | dEq=%.2f mm\n', ...
            k, result.best.tKnots(k), result.best.alpha(k), ...
            result.best.AKnots(k), result.best.dEqKnots(k) * 1e3);
    end
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Phase Events          :\n');
    fprintf('    Water phase end     : t = %.4f s (%s, Vw_rem = %.4e m^3)\n', ...
        result.phaseEvents.waterEnd.t, result.phaseEvents.waterEnd.reason, ...
        result.phaseEvents.waterEnd.VwRemaining);
    fprintf('    Air phase end       : t = %.4f s (%s, ma_rem = %.4e kg)\n', ...
        result.phaseEvents.airEnd.t, result.phaseEvents.airEnd.reason, ...
        result.phaseEvents.airEnd.maRemaining);
    fprintf('    Apex reached        : t = %.4f s (z = %.3f m)\n', ...
        result.phaseEvents.apex.t, result.phaseEvents.apex.z);
    if result.diagnostics.controlHoldActive
        fprintf('  [DIAGNOSTIC] Propulsion lasted until t=%.4f s (> tControlEnd=%.4f s);\n', ...
            result.diagnostics.tPropulsionEnd, cfgSI.tControlEnd);
        fprintf('               final knot opening was held after tControlEnd.\n');
    end
    fprintf('  Diagnostics           : solver=%s, exitflag=%d, maxViol=%.2e, maxSlew=%.3e m^2/s\n', ...
        result.diagnostics.solverUsed, result.diagnostics.exitflag, ...
        result.diagnostics.maxConstraintViolation, result.diagnostics.maxSlewRate);
    fprintf('                          ODE evals=%d, ODE failures=%d\n', ...
        result.diagnostics.nOdeEvaluations, result.diagnostics.nOdeFailures);
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Hardware & Safety Caveats:\n');
    for i = 1:numel(result.diagnostics.limitationsNote)
        fprintf('    %s\n', result.diagnostics.limitationsNote(i));
    end
    fprintf('====================================================================\n\n');
end
