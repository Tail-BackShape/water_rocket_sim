# 可変ノズル水ロケット高度最適化シミュレータ (`water_rocket_sim`)

PETボトル級の単段水ロケットについて、**初期充填水率** $f = V_{w0}/V_b$ と **噴射中のノズル有効開口面積の時間履歴** $A(t)$ を同時に最適化し、発射位置からの最大到達高度 $H_{\max}$ を求める MATLAB パッケージです。

---

## 1. ディレクトリ構成

```text
water_rocket_sim/
  runOptimization.m          % 実行入口（設定検証・最適化・レポート出力・図作成）
  +wr/
    defaultConfig.m          % デモ設定テンプレート生成
    validateConfig.m         % 入力検証・ユーザー表示単位→SI単位変換
    openingAtTime.m          % 区分線形ノズル開口補間 A(t), dEq(t)
    waterPropulsion.m        % Phase W: 水噴射フェーズ熱力学・推力計算
    airPropulsion.m          % Phase A: 圧縮空気噴射フェーズ（チョーク・非チョーク）
    rocketRHS.m              % 2次元質点運動方程式（姿勢追従近似・空気抗力）
    rocketEvents.m           % ode45 フェーズ遷移・最高点・着地イベント定義
    simulateRocket.m         % 多フェーズ連結数値積分・失敗診断
    evaluateCandidate.m      % 直接単一シューティング目的関数評価
    optimizeRocket.m         % 一定開口基準探索＋fmincon(SQP)マルチスタート最適化
    plotResults.m            % 最適解と一定開口基準の比較プロット作成
  tests/
    TestConfig.m             % 設定・単位変換・異常値拒否テスト
    TestNozzle.m             % ノズル補間・端点保持・変化速度上限テスト
    TestWaterPropulsion.m    % 水噴射フェーズ単体テスト
    TestAirPropulsion.m      % 空気噴射フェーズ・チョーク境界連続性テスト
    TestDynamics.m           % 運動方程式・姿勢遷移・対称性テスト
    TestSimulation.m         % フェーズ連結連続性・残水カットオフ・異常診断テスト
    TestOptimization.m       % 制約充足・基準解保証・再現性・可視化テスト
  docs/
    model.md                 % 数理モデル・ソルバ解説
    2026-11-06-water-rocket-optimal-control-design.md
    2026-11-06-water-rocket-matlab-implementation.md
```

---

## 2. 動作環境・Toolbox要件

- **MATLAB**（R2024a で検証済み）
  - 単体シミュレーション（`wr.simulateRocket`）およびテストの一部は MATLAB 標準機能（`ode45`, `odeset`, `runtests`）のみで動作します。
- **Optimization Toolbox（必須）**
  - `wr.optimizeRocket` / `runOptimization` における制約付き最適化（`fmincon` SQP法）に必要です。
  - 仕様に基づき、`fmincon` が存在しない環境では単純な罰則付き `fminsearch` による代替を「制約保証済み」として返さず、明示的にエラー（`wr:missingToolbox`）を報告します。
- **Global Optimization Toolbox（任意）**
  - `cfg.useGA = true` を設定した場合のみ、遺伝的アルゴリズム（`ga`）による大域探索を併用します。未導入環境でも標準のマルチスタート `fmincon` で動作します。

---

## 3. 使い方

### 3.1 デモ設定での最適化・可視化実行

MATLAB コマンドウィンドウでプロジェクトルートをカレントディレクトリにし、以下を実行します。

```matlab
[result, figs] = runOptimization();
```

### 3.2 パラメータを変更して実行（ユーザー表示単位・SI単位）

`wr.defaultConfig()` の既定値は **動作確認用のデモ専用値** です。実機解析では機体寸法・乾燥質量・ノズル限界を明示的に設定してください。計算内部はすべて SI 単位ですが、入力層（`wr.validateConfig`）では L・mm・deg・kPa（ゲージ圧）からの自動換算もサポートしています。

```matlab
cfg = wr.defaultConfig();
cfg.isDemo = false;

% ユーザー表示単位で指定する場合（対応するSIフィールドを空にする）
cfg.Vb = [];        cfg.Vb_L = 1.5;          % タンク容積 [L] -> 1.5e-3 m^3
cfg.D = [];         cfg.D_mm = 90;           % 胴体直径 [mm] -> 0.090 m
cfg.theta0 = [];    cfg.theta0_deg = 85;     % 発射角 [deg] -> rad
cfg.p0Abs = [];     cfg.pGauge = [];
cfg.pGauge_kPa = 400;                        % 初期ゲージ圧 [kPaG] -> p0Abs = pGauge + pAmb

% SI単位で直接指定する場合
cfg.mDry = 0.250;                            % 乾燥質量（駆動機構・電装込み）[kg]
cfg.Amin = pi * (0.005 / 2)^2;               % 最小有効開口面積 [m^2]
cfg.Amax = pi * (0.015 / 2)^2;               % 最大有効開口面積 [m^2]
cfg.ARateMax = 1.2e-3;                       % 最大開口面積変化速度 [m^2/s]

[result, figs] = runOptimization(cfg);
```

> **注意**: `p0Abs`（絶対圧）と `pGauge`（ゲージ圧）に矛盾する値を同時指定した場合は `wr:invalidConfig` エラーとなります。

### 3.3 再現用乱数種と最適化の中断・反復数制限

- **再現性**: `cfg.randomSeed = 42;`（既定値）によりマルチスタート初期値の乱数種が固定され、同一条件で同一の最適化結果が再現されます（呼び出し元の乱数状態は終了時に復元されます）。
- **中断方法**: 実行中の強制中断は MATLAB コンソールで `Ctrl + C` を押してください。また、`cfg.maxIter = 20;` や `cfg.nMultiStart = 1;` を指定することで計算時間を短縮できます。

### 3.4 自動テストの実行

```matlab
results = runtests('tests');
assertSuccess(results);
```

---

## 4. デモ設定における出力例と感度検証結果

デモ設定（$V_b = 1.5\,\mathrm{L}$, $m_{\mathrm{dry}} = 0.25\,\mathrm{kg}$, $D = 90\,\mathrm{mm}$, $p_{\mathrm{gauge}} = 400\,\mathrm{kPa}$, $\theta_0 = 85^\circ$, $d_{\mathrm{eq}} \in [5, 15]\,\mathrm{mm}$, $R_A = 1.2\times 10^{-3}\,\mathrm{m^2/s}$）における実行結果は以下の通りです。

- **最良一定開口基準（Baseline）**: $H_{\max} = 29.4555\,\mathrm{m}$ ($f = 0.375$, $V_{w0} = 0.563\,\mathrm{L}$, $\alpha = 1.0$)
- **最適解（Optimal）**: $H_{\max} = 29.4756\,\mathrm{m}$ ($f^* = 0.3642$, $V_{w0}^* = 0.546\,\mathrm{L}$, 到達時刻 $t_{\mathrm{apex}} = 2.4489\,\mathrm{s}$)
  - Phase W 終了（水枯渇）: $t = 0.1385\,\mathrm{s}$
  - Phase A 終了（圧力平衡）: $t = 0.1837\,\mathrm{s}$
  - 最大制約違反量: `0.00e+00`

### 数値許容誤差・ノット数・制御末尾時刻の感度解析

| 解析ケース | 設定変更内容 | 最大高度 $H_{\max}$ $[\mathrm{m}]$ | 基準設定との差 $\Delta H_{\max}$ $[\mathrm{m}]$ |
|---|---|---:|---:|
| 単体軌道積分精度検証 | `RelTol=1e-8`, `AbsTol=1e-10`（$f=0.35, \alpha=0.5$） | `28.246578` | `+4.479e-05` |
| 最適化基準設定 | `N=6`, `tControlEnd=0.30 s`, `RelTol=1e-6`, `AbsTol=1e-8` | `29.4756` | `0.0000e+00` |
| ODE許容誤差厳格化 | `RelTol=1e-7`, `AbsTol=1e-9` | `29.4756` | `-1.6499e-05` |
| 制御ノット数増加 | `nControlKnots = 8` | `29.4756` | `-2.8622e-05` |
| 制御時間延長 | `tControlEnd = 0.45 s` | `29.4756` | `+4.0362e-06` |

---

## 5. 安全上の注意と残存制限事項（実機投入前の確認事項）

1. **圧力容器・継手・弁の耐圧安全性**
   - 本シミュレータはPETボトル・継手・可変ノズル機構の耐圧強度や安全率を**一切推定・保証しません**。使用者は実機の耐圧試験および安全な運用上限圧力を別途確認してください。
2. **姿勢安定性の近似**
   - 本モデルは推力方向が速度ベクトルに理想追従する2次元質点近似です。実機の静安定マージン（重心・圧力中心位置）、動安定性、ランチャー離脱時のチップオフ、横風の影響は別途検討が必要です。
3. **可変ノズル機構の実機校正作業（今後のステップ）**
   - モデル内の $A(t)$ は縮流・摩擦損失を含んだ**有効開口面積**です。実機機構が決まった段階で、各開度における流量係数 $C_d(\alpha)$・速度係数 $C_v(\alpha)$ を静止推力試験で同定してください。
   - 内圧負荷下でのバルブ駆動トルク・推力、アクチュエータ応答遅れ、および駆動系による乾燥質量 $m_{\mathrm{dry}}$ の増加を実測し、`cfg` に反映して再最適化を実施してください。
