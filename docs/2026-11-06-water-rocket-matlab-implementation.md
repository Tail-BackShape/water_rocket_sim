# Water Rocket MATLAB Optimal Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MATLABで、PETボトル水ロケットの最適初期水量と実行可能な可変ノズル開口スケジュールを求め、最大到達高度を報告する。

**Architecture:** `+wr` MATLABパッケージに物理式、フェーズ別ODE、イベント積分、制約付き直接シューティング最適化を分離する。`runOptimization.m`から入力検証・基準ケース・最適化・検証図を実行する。未定のノズル機構を固定せず、有効開口面積と最大変更速度をパラメータ化する。

**Tech Stack:** MATLAB、標準 `ode45`／`odeset`／`runtests`、Optimization Toolbox の `fmincon`（最適化に必要）、任意の Global Optimization Toolbox `ga`。Simulink、CFD、Python依存は不要。

**Spec:** `docs/superpowers/specs/2026-11-06-water-rocket-optimal-control-design.md`

## Global Constraints

- まだ実装していない。実装着手前に本指示書とSpecを利用者に確認してもらう。
- 物理モデルは2D質点、風なし、姿勢追従近似、水→空気→慣性飛行。ランチャー拘束は必須にしない。
- 最適化変数は初期水量とノズル開口履歴のみ。初期圧力・発射角度は入力条件。
- 計算内部はSI単位。入力UIでのみL・mm・度・ゲージ圧を変換する。
- 有効開口の上下限・開口変更速度を守る。指定された制御の実機実現性は**機構選定後の確認事項**。
- 最適化器が失敗しても最良の実行可能な一定開口基準解は保持する。無効候補を成功扱いしない。
- 初期圧力の安全性は使用者が別途判断する。ソルバが圧力容器の安全性を保証しない。
- `fmincon` がない場合はその旨を明示する。`fminsearch`に単純な罰則を加えただけの結果を「制約保証済み」と表示しない。
- 各小タスクはまず失敗する自動テストを書き、実装し、`runtests`で検証してから次へ進む。

## Review Focus

1. 初期圧力が外気圧以下、またはノズル閉鎖で離陸不能：`TestSimulation.testNoLiftoffIsNotSuccess`が診断を検証する。
2. 水が残ったまま圧力が外気圧に達する：`TestSimulation.testResidualWaterAtPressureCutoff`が残水質量とフェーズ移行を検証する。
3. 初期鉛直速度ゼロを最高点と誤検出：`TestSimulation.testApexNotAtStart`が検証する。
4. ノズル開口の跳び・速度制約違反：`TestNozzle.testSlewBound`が全区間の勾配を検証する。
5. 最適化が一定開口基準より悪い局所解に収束：`TestOptimization.testNeverBelowBaseline`が最終採用規則を検証する。

---

## ファイル構成と公開インターフェース

```text
water_rocket_matlab/
  runOptimization.m
  +wr/
    defaultConfig.m
    validateConfig.m
    openingAtTime.m
    waterPropulsion.m
    airPropulsion.m
    rocketRHS.m
    rocketEvents.m
    simulateRocket.m
    evaluateCandidate.m
    optimizeRocket.m
    plotResults.m
  tests/
    TestConfig.m
    TestNozzle.m
    TestWaterPropulsion.m
    TestAirPropulsion.m
    TestDynamics.m
    TestSimulation.m
    TestOptimization.m
  README.md
  docs/
    model.md
```

推奨の関数シグネチャ（戻り値はMATLABのstruct中心）:

| 関数 | 入力 | 出力／責務 |
|---|---|---|
| `wr.defaultConfig()` | なし | `cfg`: 物性・入力単位変換・計算設定テンプレート |
| `wr.validateConfig(cfg)` | `cfg` | `cfgSI`: 型・範囲・必須値チェック済み設定 |
| `wr.openingAtTime(t,alpha,cfgSI)` | 秒、ノット値、設定 | `[A,dEq]`: 有効開口面積・等価直径 |
| `wr.waterPropulsion(t,y,alpha,cfgSI)` | 水噴射状態 | `prop`: 圧力・温度・推力・`dVw`・`dMa` |
| `wr.airPropulsion(t,y,alpha,cfgSI,phase0)` | 空気噴射状態と開始時の気体値 | `prop`: 圧力・温度・推力・`dVw`・`dMa` |
| `wr.rocketRHS(t,y,phase,alpha,cfgSI,phase0)` | 状態とフェーズ `"water"/"air"/"coast"` | `dy`: 6要素列ベクトル |
| `wr.rocketEvents(t,y,phase,alpha,cfgSI,phase0)` | 同上 | `[value,isterminal,direction]`: イベント値 |
| `wr.simulateRocket(f,alpha,cfgSI)` | 充填率とノット値 | `sim`: 最高点・全時系列・イベント・状態コード |
| `wr.evaluateCandidate(q,cfgSI)` | `q=[f,alpha(:).']` | `[cost,sim]`: `cost=-Hmax`（有効候補） |
| `wr.optimizeRocket(cfgSI)` | 設定 | `result`: 最適候補・一定開口基準・診断 |
| `wr.plotResults(result,cfgSI)` | 結果 | `figs`: 図ハンドル・必要な図の作成 |

`cfgSI`に `Vb,mDry,D,p0Abs,theta0,T0,pAmb,rhoAir,rhoW,Cd,gamma,Rair,g,Amin,Amax,ARateMax,tControlEnd,nControlKnots,fillMin,fillMax,vEpsilon,relTol,absTol,maxStep,maxFlightTime` を保持する。MATLABの既定値はユーザー入力の代用にしない。

`sim.status` は最低限 `"success"`, `"noLiftoff"`, `"groundImpact"`, `"timeout"`, `"numericalFailure"`, `"invalidInput"` の列挙可能な文字列とする。`sim.phaseEvents`にイベント時刻・タイプ・残状態を記録する。関数間で状態ベクトル `[x,z,vx,vz,Vw,ma]` の順序を固定する。

## Task 1: 設定・単位と検証

**Files:** `+wr/defaultConfig.m`, `+wr/validateConfig.m`, `tests/TestConfig.m`, `README.md`。

**Interfaces:** `cfgSI = wr.validateConfig(cfg)`。`cfg`はユーザー表示単位かSIを明示する一貫した形式に限定する。圧力は絶対圧・ゲージ圧を混同させない。

- [ ] **Step 1: `TestConfig` の失敗テストを書く。** `testPressureConversion` は `p0Abs = pGauge + pAmb` を `verifyEqual(...,'AbsTol',1e-9)` で確認。`testRejectNearlyFull` は `fillMax>=1` を拒否。`testRejectBadNozzleBounds` は `Amax<=Amin` を拒否。`testMissingDryMass` は空の質量を拒否。
- [ ] **Step 2: `runtests('tests/TestConfig.m')` を実行し、未実装の失敗を確認する。**
- [ ] **Step 3: `defaultConfig` と `validateConfig` を最小実装する。** 必須の実機値を勝手な安全値で埋めず、検証可能なサンプル値はREADMEで「デモ専用」と明示。
- [ ] **Step 4: 同じテストを再実行し、全PASSを確認する。**
- [ ] **Step 5: Gitリポジトリで作業する場合、テスト込みでコミットする。** 例: `git commit -m "feat: validate MATLAB rocket configuration"`。

## Task 2: 時間依存ノズル制御と線形制約

**Files:** `+wr/openingAtTime.m`, `tests/TestNozzle.m`。

**Interfaces:** `A(t)=Amin+(Amax-Amin)*interp1(tKnots,alpha,t,'linear')`（領域外は端値を保持）。`dEq=sqrt(4*A/pi)`。`alpha`は `nControlKnots` 個。

- [ ] **Step 1: 失敗テストを書く。** `testConstantKnots` は全点同じなら全時刻で一定。`testLinearMidpoint` は中点補間値が端点の算術平均。`testEndpointHold` は制御末尾後に最終値を保持。`testSlewBound` は全区間で `abs(diff(A))./diff(tKnots) <= ARateMax + tolerance`。`testNoOvershoot` は`[Amin,Amax]`から逸脱しない。
- [ ] **Step 2: `runtests('tests/TestNozzle.m')` で失敗を確認する。**
- [ ] **Step 3: `openingAtTime` と、最適化用の上下限・隣接差分線形制約行列構築を実装する。** 後者は `optimizeRocket.m` の内部ヘルパーまたは局所関数とする。
- [ ] **Step 4: テストを再実行してPASSを確認する。**
- [ ] **Step 5: コミットする。**

## Task 3: 水噴射の熱力学・流量・推力

**Files:** `+wr/waterPropulsion.m`, `tests/TestWaterPropulsion.m`。

**Interfaces:** `prop` fields = `p,T,thrust,dVw,dMa,mDotOut`。`dMa=0`。仕様書の `p0*(Va0/(Vb-Vw))^gamma`, `dVw=-A*sqrt(2*max(p-pAmb,0)/rhoW)`, `thrust=2*A*max(p-pAmb,0)` を使う。

- [ ] **Step 1: 失敗テストを書く。** `testInitialPressure` は初期水量で`p=p0Abs`。`testPressureDrops` は `Vw` 減少後に `p` が下がる。`testAreaScaling` は同状態で開口2倍なら流量・推力2倍。`testNoReverseFlow` は `p<=pAmb` で流量・推力がゼロ。
- [ ] **Step 2: `runtests('tests/TestWaterPropulsion.m')` で失敗を確認する。**
- [ ] **Step 3: `waterPropulsion` を実装する。** `ma` と総質量の連続性は後段の動力学で確認する。
- [ ] **Step 4: テストを再実行してPASSを確認する。**
- [ ] **Step 5: コミットする。**

## Task 4: 空気噴射のチョーク・非チョークモデル

**Files:** `+wr/airPropulsion.m`, `tests/TestAirPropulsion.m`。

**Interfaces:** `phase0`に `p1,T1,ma1` を保持。`prop` は Task 3 と同じフィールド。`dVw=0`, `dMa=-mDotOut`。臨界圧比は `(2/(gamma+1))^(gamma/(gamma-1))`。式はSpec第7節。

- [ ] **Step 1: 失敗テストを書く。** `testChokedFlowPositive`, `testUnchokedFlowPositive`, `testNoFlowAtAmbient`, `testPressureDecreasesWithGasMass`, `testChokeBoundaryContinuous`。境界の両側を相対差の十分小さい値で評価し、質量流量と推力の有限性・連続性を検証する。
- [ ] **Step 2: `runtests('tests/TestAirPropulsion.m')` で失敗を確認する。**
- [ ] **Step 3: `airPropulsion` を実装する。** 端点近傍の根号内負値は丸め誤差として限定的にクリップし、真に無効な状態を隠さない。
- [ ] **Step 4: テストを再実行してPASSを確認する。**
- [ ] **Step 5: コミットする。**

## Task 5: 2次元運動方程式・姿勢近似

**Files:** `+wr/rocketRHS.m`, `tests/TestDynamics.m`。

**Interfaces:** `dy` は6要素列ベクトル。`rocketRHS`は選択フェーズの `prop` を呼び、推力方向を発射初期ベクトルまたは速度単位ベクトルとする。抗力は `-0.5*rhoAir*Cd*pi*D^2/4*norm(v)*v`。

- [ ] **Step 1: 失敗テストを書く。** `testZeroSpeedUsesLaunchAngle`、`testVelocityAlignmentAfterThreshold`、`testDragOpposesVelocity`、`testVerticalSymmetry`（角度90度・横速度ゼロで横加速度ゼロ）、`testMassUsesRemainingPropellant`。
- [ ] **Step 2: `runtests('tests/TestDynamics.m')` で失敗を確認する。**
- [ ] **Step 3: `rocketRHS` を実装する。** `vEpsilon`前後で異常な不連続振動を起こさないか確認する。
- [ ] **Step 4: テストを再実行してPASSを確認する。**
- [ ] **Step 5: コミットする。**

## Task 6: フェーズ連結・最高点イベントと失敗診断

**Files:** `+wr/rocketEvents.m`, `+wr/simulateRocket.m`, `tests/TestSimulation.m`。

**Interfaces:** `sim=wr.simulateRocket(f,alpha,cfgSI)`。Phase W→A/C→C の切替えは `ode45` の終端イベントで行う。フェーズ開始時の `p1,T1,ma1` を記録。`vz`が正から負へ横断したイベントを最高点とする。各フェーズの出力を同じ時間順で結合する（重複時刻を二重登録しない）。

- [ ] **Step 1: 失敗テストを書く。** `testWaterToAirContinuity` は位置・速度・総質量が境界前後で一致。`testResidualWaterAtPressureCutoff` は `p==pAmb` で水が残る設定からPhase Cへ移る。`testApexNotAtStart` は `t=0,vz=0` で高度ゼロの誤判定をしない。`testNoLiftoffIsNotSuccess`、`testClosedNozzleTimeoutOrFailure`、`testStateNonnegative`、`testMaxTimeTimeout`を追加。
- [ ] **Step 2: `runtests('tests/TestSimulation.m')` で失敗を確認する。**
- [ ] **Step 3: `rocketEvents` と `simulateRocket` を実装する。** イベントが近接／同時発生するとき、残水・内部圧による決定を優先。最高点イベントは全推進フェーズでも監視する。必要なら発射直後の上昇成立判定を別途持つ。
- [ ] **Step 4: テストを再実行してPASSを確認する。** `RelTol`と `AbsTol`の変更による高度差も記録。
- [ ] **Step 5: コミットする。**

## Task 7: 初期水量＋可変ノズルの最適化

**Files:** `+wr/evaluateCandidate.m`, `+wr/optimizeRocket.m`, `tests/TestOptimization.m`。

**Interfaces:** `q=[f,alpha_1,...,alpha_N]`、`cost=-sim.Hmax`（有効時）。最良一定開口を初期値・安全な出力候補として保存。`fmincon`への制約は領域の上下限とノット差分の線形不等式。最終結果 `result.best`, `result.baseline`, `result.diagnostics`。

- [ ] **Step 1: 失敗テストを書く。** `testCandidateBounds`、`testSlewFeasibility`、`testOptimizesWaterFraction`（水量が決定変数に含まれる）、`testNeverBelowBaseline`、`testInvalidCandidateIsNotOptimal`、`testRepeatabilityWithSeed`（同乱数種で同結果または許容近似）。
- [ ] **Step 2: `runtests('tests/TestOptimization.m')` で失敗を確認する。**
- [ ] **Step 3: `evaluateCandidate` と `optimizeRocket` を実装する。** 予備の一定開口探索→最良一定開口を含む複数初期値→`fmincon` SQP→最良実行可能候補の採用。`ga` は機能検出で追加できる任意ルートとし、必須にしない。
- [ ] **Step 4: テストを再実行してPASSを確認する。** 単一初期値の局所解を大域最適と誤記しない。
- [ ] **Step 5: コミットする。**

## Task 8: 実行入口、可視化と最終検証

**Files:** `runOptimization.m`, `+wr/plotResults.m`, `README.md`, `docs/model.md`, `tests/TestOptimization.m`。

**Interfaces:** `runOptimization` は `cfg`を準備・検証し、`result=wr.optimizeRocket(cfgSI)`、`wr.plotResults(result,cfgSI)` を呼ぶ。描画は最適結果・一定開口基準を識別可能にする。

- [ ] **Step 1: レポート項目テストを書く。** `testResultContainsPhaseEvents`, `testReportHasConstraintViolation`, `testBaselineComparisonFinite`, `testNoPercentWhenBaselineZero`。図の軸がSIに整合し、`A(t)`・`dEq(t)`・高度・圧力・推力・残水量・速度を出すことも検証。
- [ ] **Step 2: テストで未実装の失敗を確認する。**
- [ ] **Step 3: 実行入口・図・READMEを実装する。** READMEに入力変換、Toolbox要件、圧力容器安全、モデル近似、出力例、最適化中断方法、再現用乱数種を記載。
- [ ] **Step 4: `runtests('tests')` 全件を実行する。** 許容誤差を1段階厳しくした試験、ノット数を増やした試験、`tControlEnd`延長試験を行い、それぞれの高度差を最終レポートへ記載する。
- [ ] **Step 5: コミットする。** 最後のREADMEに残存制限事項と次の実機校正作業を書く。

## 受入判定

1. `runtests('tests')` が全PASSし、無効入力で自然なエラーメッセージになる。
2. 最適化結果は初期水量と全制御点、実時間 `A*(t)`、等価直径 `dEq*(t)`、最大高度を返す。
3. 制御点間の開口変化速度、面積範囲、水量範囲がすべて満たされる。
4. 物理モデルは水→空気→慣性の切替えで質量・速度が不連続にならない。
5. 初期 `vz=0` を最高点と誤判定しない。
6. 最終採用高度は記録済み最良一定開口基準ケース以上。
7. ノット数・ODE許容誤差・制御時間に対する感度と最適化終了状態が提示される。
8. レポートは姿勢安定性／駆動機構の実現可能性／圧力容器安全の未検証事項を明記する。

## 実装開始前の確認事項

使用者は本計画とSpecを確認し、実装開始を別途指示する。未入力の機体寸法・圧力・ノズル限界は実装時の設定値として埋められるため、今の時点で仮決めしなくてよい。MATLABライセンスにOptimization Toolboxが含まれない場合は、先に実行可能な別ソルバを合意する。

