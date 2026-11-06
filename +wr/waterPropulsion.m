function prop = waterPropulsion(t, y, alpha, cfgSI)
%WATERPROPULSION 水噴射フェーズ (Phase W) の状態量・流量・推力を計算する
%   状態ベクトル y = [x; z; vx; vz; Vw; ma] に対し、空気の断熱膨張と
%   ベルヌーイ式による水噴射モデルを評価する。

    % イベント検出までの微小なステップはみ出しを物理範囲内にクランプ
    VwRaw = y(5);
    Vw = min(max(VwRaw, 0.0), cfgSI.Vb * (1.0 - 1e-12));
    ma = max(y(6), 0.0);

    % 一定空気質量 ma = ma0 から初期空気体積 Va0 を復元
    Va0 = ma * cfgSI.Rair * cfgSI.T0 / cfgSI.p0Abs;
    Va = max(cfgSI.Vb - Vw, 1e-15);

    expansionRatio = max(Va0 / Va, 0.0);
    p = cfgSI.p0Abs * (expansionRatio ^ cfgSI.gamma);
    T = cfgSI.T0 * (expansionRatio ^ (cfgSI.gamma - 1.0));

    deltaP = max(p - cfgSI.pAmb, 0.0);
    [A, dEq] = wr.openingAtTime(t, alpha, cfgSI);

    if VwRaw <= 0.0 || deltaP <= 0.0 || A <= 0.0
        uw = 0.0;
        dVw = 0.0;
        mDotOut = 0.0;
        thrust = 0.0;
    else
        uw = sqrt(2.0 * deltaP / cfgSI.rhoW);
        dVw = -A * uw;
        mDotOut = cfgSI.rhoW * A * uw;
        thrust = 2.0 * A * deltaP;
    end

    prop = struct( ...
        'p', p, ...
        'T', T, ...
        'thrust', thrust, ...
        'dVw', dVw, ...
        'dMa', 0.0, ...
        'mDotOut', mDotOut, ...
        'A', A, ...
        'dEq', dEq, ...
        'uw', uw, ...
        'deltaP', deltaP);
end
