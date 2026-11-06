# 可変ノズル水ロケット高度最適化 — 数理モデル・ソルバ解説 (`docs/model.md`)

本書は `+wr` パッケージに実装された2次元質点・多フェーズ水ロケット物理モデルおよび制約付き直接単一シューティング最適化の解説資料です。

---

## 1. 座標系と状態変数

発射位置を原点 $(x, z) = (0, 0)$ とし、水平前方 $x$、鉛直上向き $z$ の2次元直交座標系を採用します。計算内部はすべて **SI単位系** で統一しています。

状態ベクトル $\boldsymbol{y} \in \mathbb{R}^6$ は以下の順序で固定されています。

$$
\boldsymbol{y} = \begin{bmatrix} x & z & v_x & v_z & V_w & m_a \end{bmatrix}^\mathsf{T}
$$

- $x, z$ : 水平位置・高度 $[\mathrm{m}]$
- $v_x, v_z$ : 水平速度・鉛直速度 $[\mathrm{m/s}]$
- $V_w$ : タンク内残存水体積 $[\mathrm{m^3}]$
- $m_a$ : タンク内残存空気質量 $[\mathrm{kg}]$

機体総質量 $m(t)$ は乾燥機体質量 $m_{\mathrm{dry}}$（ノズル駆動機構・搭載電装を含む）と推進剤質量の和で表されます。

$$
m(t) = m_{\mathrm{dry}} + \rho_w V_w(t) + m_a(t)
$$

---

## 2. 並進運動方程式と姿勢追従近似 (`+wr/rocketRHS.m`)

無風条件下の並進運動方程式は以下の通りです。

$$
\dot{x} = v_x, \qquad \dot{z} = v_z
$$

$$
\dot{\boldsymbol{v}} = \frac{F_T(t)\boldsymbol{e}_T - \frac{1}{2}\rho_{\mathrm{air}} C_D S |\boldsymbol{v}|\boldsymbol{v}}{m(t)} - g\begin{bmatrix} 0 \\ 1 \end{bmatrix}, \qquad S = \frac{\pi D^2}{4}
$$

推力方向単位ベクトル $\boldsymbol{e}_T$ は、発射直後のゼロ割り回避閾値 $v_\epsilon > 0$ を用いて以下のように定義します。

$$
\boldsymbol{e}_T = \begin{cases}
[\cos\theta_0,\, \sin\theta_0]^\mathsf{T} & (|\boldsymbol{v}| < v_\epsilon) \\
\boldsymbol{v} / |\boldsymbol{v}| & (|\boldsymbol{v}| \ge v_\epsilon)
\end{cases}
$$

---

## 3. 時間依存ノズル開口スケジュール (`+wr/openingAtTime.m`)

可変ノズル機構に依存しない **有効開口面積** $A(t) \in [A_{\min}, A_{\max}]$ を制御入力とします。制御区間 $[0, t_{\mathrm{controlEnd}}]$ を $N$ 個の等間隔ノット $t_1 = 0, \dots, t_N = t_{\mathrm{controlEnd}}$ に分割し、正規化開度 $\alpha_i \in [0, 1]$ の区分線形補間で定義します。

$$
A(t) = A_{\min} + (A_{\max} - A_{\min})\alpha(t), \qquad d_{\mathrm{eq}}(t) = \sqrt{\frac{4A(t)}{\pi}}
$$

- $t < 0$ では先頭値 $\alpha_1$ を保持、$t > t_{\mathrm{controlEnd}}$ では最終値 $\alpha_N$ を保持します（スプライン補間によるオーバーシュートは使用しません）。
- 隣接ノット間の変化速度は機構上限 $R_A = \mathtt{ARateMax}$ $[\mathrm{m^2/s}]$ により以下の線形不等式で制約されます。

$$
|\alpha_{i+1} - \alpha_i| \le \frac{R_A (t_{i+1} - t_i)}{A_{\max} - A_{\min}} \quad (i = 1, \dots, N-1)
$$

---

## 4. 推進・飛行フェーズとイベント遷移 (`+wr/simulateRocket.m`)

### 4.1 Phase W: 水噴射フェーズ (`+wr/waterPropulsion.m`)

初期充填率 $f = V_{w0} / V_b \in [f_{\min}, f_{\max}]$ に対し、初期空気体積 $V_{a0} = V_b - V_{w0}$、初期空気質量 $m_{a0} = p_{0,\mathrm{abs}} V_{a0} / (R_{\mathrm{air}} T_0)$ とします。水噴射中は $m_a = m_{a0}$ 一定であり、空気の断熱膨張と非圧縮ベルヌーイ流出を仮定します。

$$
V_a = V_b - V_w, \qquad p = p_{0,\mathrm{abs}}\left(\frac{V_{a0}}{V_a}\right)^\gamma, \qquad T = T_0\left(\frac{V_{a0}}{V_a}\right)^{\gamma-1}
$$

$$
\Delta p = \max(p - p_{\mathrm{amb}}, 0), \qquad u_w = \sqrt{\frac{2\Delta p}{\rho_w}}, \qquad \dot{V}_w = -A(t) u_w, \qquad F_T = 2A(t)\Delta p
$$

- **終了イベント**:
  1. $V_w = 0$（水枯渇）: $p > p_{\mathrm{amb}}$ なら開始状態 $(p_1, T_1, m_{a1})$ を保存して Phase A へ遷移。それ以外は Phase C へ遷移。
  2. $p = p_{\mathrm{amb}}$（圧力平衡）: 残存水 $V_w > 0$ を機体質量に含めたまま Phase C へ直接遷移（Phase A はスキップ）。
  3. $v_z = 0$（上昇開始後の正→負横断）: 最高点到達として終了。
  4. $z = 0$（地面落下）: 失敗診断として終了。

### 4.2 Phase A: 圧縮空気噴射フェーズ (`+wr/airPropulsion.m`)

水が全て排出された時点の気体状態 $(p_1, T_1, m_{a1})$ を基準に、剛体タンクからの断熱ブローダウンと等エントロピーノズル流出を解きます。

$$
p = p_1\left(\frac{m_a}{m_{a1}}\right)^\gamma, \qquad T = T_1\left(\frac{m_a}{m_{a1}}\right)^{\gamma-1}, \qquad r = \frac{p_{\mathrm{amb}}}{p}, \qquad r_{\mathrm{crit}} = \left(\frac{2}{\gamma+1}\right)^{\frac{\gamma}{\gamma-1}}
$$

- **チョーク時 ($r \le r_{\mathrm{crit}}$)**:
  $$\dot{m}_{\mathrm{out}} = A(t)p\sqrt{\frac{\gamma}{R_{\mathrm{air}}T}}\left(\frac{2}{\gamma+1}\right)^{\frac{\gamma+1}{2(\gamma-1)}}, \quad p_e = p r_{\mathrm{crit}}, \quad T_e = \frac{2T}{\gamma+1}, \quad u_e = \sqrt{\gamma R_{\mathrm{air}} T_e}$$
- **非チョーク時 ($r_{\mathrm{crit}} < r < 1$)**:
  $$\dot{m}_{\mathrm{out}} = A(t)p\sqrt{\frac{\gamma}{R_{\mathrm{air}}T}}\sqrt{\frac{2}{\gamma-1}\left(r^{2/\gamma} - r^{(\gamma+1)/\gamma}\right)}, \quad p_e = p_{\mathrm{amb}}, \quad u_e = \sqrt{\frac{2\gamma R_{\mathrm{air}}T}{\gamma-1}\left(1 - r^{\frac{\gamma-1}{\gamma}}\right)}$$
- **推力および質量変化**:
  $$\dot{m}_a = -\dot{m}_{\mathrm{out}}, \qquad F_T = \dot{m}_{\mathrm{out}} u_e + (p_e - p_{\mathrm{amb}})A(t)$$

### 4.3 Phase C: 慣性飛行フェーズ

$F_T = 0, \dot{V}_w = 0, \dot{m}_a = 0$ として重力と空気抗力のみを積分し、`ode45` の Events 機能で $v_z = 0$（正→負）となる最高点時刻 $t_{\mathrm{apex}}$ と最高到達高度 $H_{\max} = z(t_{\mathrm{apex}})$ を精密に特定します。

---

## 5. 直接単一シューティング最適化 (`+wr/optimizeRocket.m`)

決定変数ベクトル $\boldsymbol{q} = [f, \alpha_1, \dots, \alpha_N]^\mathsf{T} \in \mathbb{R}^{N+1}$ に対し、目的関数 $J(\boldsymbol{q}) = -H_{\max}(\boldsymbol{q})$ を最小化します。

1. **一定開口基準探索（Baseline）**: 複数の充填率 $f$ と一定開度 $\alpha_1 = \dots = \alpha_N = c$ の格子探索を行い、最良の実行可能解を `result.baseline` として保存します。
2. **マルチスタート局所最適化**: 最良一定開口解およびスルーレート制約を満たすランプ型初期推定から `fmincon`（SQP法）を実行します。
3. **安全採用規則**: 最終採用解 `result.best` は常に厳密に実行可能な候補のみから選ばれ、必ず $H_{\max}(\boldsymbol{q}^*) \ge H_{\max}(\boldsymbol{q}_{\mathrm{baseline}})$ を満たします。

---

## 6. モデルの限界と実機安全上の注意

- 本モデルは姿勢が速度ベクトルに理想追従する2次元質点近似であり、空力静安定・動安定、フィン設計、横風応答は保証しません。
- $A(t)$ は流量損失係数を含んだ有効開口面積であり、実機バルブの幾何開口面積への変換には開度別流量係数 $C_d(\alpha)$ の実測校正が必要です。
- 水圧・空気圧負荷下でのアクチュエータ必要推力・トルクおよび応答遅れは別途ベンチ試験で確認してください。
- **圧力容器・継手・ノズル弁の耐圧安全性は本ソフトウェアでは一切保証しません。**
