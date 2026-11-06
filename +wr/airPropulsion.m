function prop = airPropulsion(t, y, alpha, cfgSI, phase0)
%AIRPROPULSION 空気噴射フェーズ (Phase A) の状態量・流量・推力を計算する
%   開始状態 phase0 (p1, T1, ma1) に基づき、剛体タンクからの断熱ブローダウンと
%   等エントロピーノズル流出 (チョーク / 非チョーク) を評価する。

    if nargin < 5 || ~isstruct(phase0)
        error('wr:invalidPhase0', 'Phase A requires phase0 struct with fields p1, T1, ma1.');
    end

    maRaw = y(6);
    ma = max(maRaw, 0.0);
    ma1 = max(phase0.ma1, 1e-15);

    massRatio = max(ma / ma1, 0.0);
    g = cfgSI.gamma;

    p = phase0.p1 * (massRatio ^ g);
    T = max(phase0.T1 * (massRatio ^ (g - 1.0)), 1e-9);

    [A, dEq] = wr.openingAtTime(t, alpha, cfgSI);

    rCrit = (2.0 / (g + 1.0)) ^ (g / (g - 1.0));
    r = cfgSI.pAmb / max(p, 1e-15);

    if maRaw <= 0.0 || p <= cfgSI.pAmb * (1.0 + 1e-14) || r >= 1.0 - 1e-14 || A <= 0.0
        isChoked = false;
        mDotOut = 0.0;
        pe = cfgSI.pAmb;
        ue = 0.0;
        thrust = 0.0;
    elseif r <= rCrit
        % チョーク流れ
        isChoked = true;
        chokeFactor = (2.0 / (g + 1.0)) ^ ((g + 1.0) / (2.0 * (g - 1.0)));
        mDotOut = A * p * sqrt(g / (cfgSI.Rair * T)) * chokeFactor;
        pe = p * rCrit;
        Te = T * (2.0 / (g + 1.0));
        ue = sqrt(g * cfgSI.Rair * Te);
        thrust = mDotOut * ue + (pe - cfgSI.pAmb) * A;
    else
        % 非チョーク亜音速流れ
        isChoked = false;
        radFlow = (2.0 / (g - 1.0)) * (r^(2.0 / g) - r^((g + 1.0) / g));
        radFlow = max(radFlow, 0.0); % r=1 近傍の丸め誤差保護
        mDotOut = A * p * sqrt(g / (cfgSI.Rair * T)) * sqrt(radFlow);
        pe = cfgSI.pAmb;
        radVel = (2.0 * g * cfgSI.Rair * T / (g - 1.0)) * (1.0 - r^((g - 1.0) / g));
        radVel = max(radVel, 0.0);
        ue = sqrt(radVel);
        thrust = mDotOut * ue;
    end

    prop = struct( ...
        'p', p, ...
        'T', T, ...
        'thrust', thrust, ...
        'dVw', 0.0, ...
        'dMa', -mDotOut, ...
        'mDotOut', mDotOut, ...
        'A', A, ...
        'dEq', dEq, ...
        'pe', pe, ...
        'ue', ue, ...
        'isChoked', isChoked);
end
