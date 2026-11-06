function cfg = defaultConfig()
%DEFAULTCONFIG 水ロケット最適化のデモ用既定設定構造体を返す
%   注意: 機体寸法・質量・圧力・ノズル範囲はすべて動作確認用のデモ値であり、
%   実機の耐圧安全性や機構の成立性を保証しない。

    cfg = struct();
    cfg.isDemo = true; % デモ設定フラグ

    % --- 機体諸元 (SI単位、および任意のユーザー表示単位フィールド) ---
    cfg.Vb = 1.5e-3;          % タンク内容積 [m^3]
    cfg.Vb_L = [];            % タンク内容積 [L] (Vbが空のとき使用)
    cfg.mDry = 0.250;         % 乾燥質量 (ノズル機構・電装含む) [kg]
    cfg.D = 0.090;            % 胴体最大直径 [m]
    cfg.D_mm = [];            % 胴体最大直径 [mm] (Dが空のとき使用)

    % --- 発射条件・初期状態 ---
    cfg.pAmb = 101325.0;      % 外気絶対圧 [Pa]
    cfg.pGauge = 400000.0;    % 初期タンクゲージ圧 [Pa]
    cfg.pGauge_kPa = [];      % 初期タンクゲージ圧 [kPa] (pGauge, p0Absが空のとき使用)
    cfg.p0Abs = [];           % 初期タンク絶対圧 [Pa] (空なら pGauge + pAmb)
    cfg.theta0 = deg2rad(85); % 発射角 [rad]
    cfg.theta0_deg = [];      % 発射角 [deg] (theta0が空のとき使用)
    cfg.T0 = 293.15;          % 初期タンク空気温度 [K]

    % --- 物理定数・環境 ---
    cfg.rhoAir = 1.204;       % 外気密度 [kg/m^3]
    cfg.rhoW = 998.2;         % 水の密度 [kg/m^3]
    cfg.Cd = 0.45;            % 抗力係数 [-]
    cfg.gamma = 1.4;          % 空気の比熱比 [-]
    cfg.Rair = 287.05;        % 空気の比気体定数 [J/(kg*K)]
    cfg.g = 9.80665;          % 重力加速度 [m/s^2]

    % --- 可変ノズル制約 ---
    cfg.Amin = pi * (0.005 / 2)^2; % 最小有効開口面積 [m^2]
    cfg.Amax = pi * (0.015 / 2)^2; % 最大有効開口面積 [m^2]
    cfg.dMin_mm = [];         % 最小等価直径 [mm] (Aminが空のとき使用)
    cfg.dMax_mm = [];         % 最大等価直径 [mm] (Amaxが空のとき使用)
    cfg.ARateMax = 1.2e-3;    % 最大開口面積変化速度 [m^2/s]

    % --- 制御パラメータ化 ---
    cfg.tControlEnd = 0.30;   % 制御点の末尾時刻 [s]
    cfg.nControlKnots = 6;    % 制御ノット点数 [-]
    cfg.fillMin = 0.10;       % 初期水充填率の下限 [-]
    cfg.fillMax = 0.65;       % 初期水充填率の上限 [-]

    % --- 数値積分・最適化設定 ---
    cfg.vEpsilon = 0.20;      % 速度方向追従の遷移速度閾値 [m/s]
    cfg.relTol = 1e-6;        % ODE 相対許容誤差 [-]
    cfg.absTol = 1e-8;        % ODE 絶対許容誤差 [-]
    cfg.maxStep = 0.01;       % ODE 最大ステップ幅 [s]
    cfg.maxFlightTime = 15.0; % 最大飛行積分時間 [s]
    cfg.randomSeed = 42;      % 再現用乱数シード
    cfg.nMultiStart = 3;      % マルチスタート初期値数 [-]
    cfg.useGA = false;        % Global Optimization Toolbox (ga) 併用フラグ
end
