function dy = rocketRHS(t, y, phase, alpha, cfgSI, phase0)
%ROCKETRHS 2次元質点モデルの状態微分 dy = [vx; vz; ax; az; dVw; dMa] を返す
%   フェーズ "water", "air", "coast" に応じた推力と空気抗力・重力を合算する。

    vx = y(3);
    vz = y(4);
    Vw = max(y(5), 0.0);
    ma = max(y(6), 0.0);

    switch lower(char(phase))
        case 'water'
            prop = wr.waterPropulsion(t, y, alpha, cfgSI);
            thrust = prop.thrust;
            dVw = prop.dVw;
            dMa = prop.dMa;
        case 'air'
            prop = wr.airPropulsion(t, y, alpha, cfgSI, phase0);
            thrust = prop.thrust;
            dVw = prop.dVw;
            dMa = prop.dMa;
        case 'coast'
            thrust = 0.0;
            dVw = 0.0;
            dMa = 0.0;
        otherwise
            error('wr:invalidPhase', 'Unknown flight phase "%s".', char(phase));
    end

    % 瞬時総質量
    mTot = cfgSI.mDry + cfgSI.rhoW * Vw + ma;

    % 速度大きさと推力単位ベクトル
    vMag = hypot(vx, vz);
    if vMag < cfgSI.vEpsilon
        eTx = cos(cfgSI.theta0);
        eTz = sin(cfgSI.theta0);
    else
        eTx = vx / vMag;
        eTz = vz / vMag;
    end

    % 空気抗力成分
    S = pi * (cfgSI.D ^ 2) / 4.0;
    dragCoeff = 0.5 * cfgSI.rhoAir * cfgSI.Cd * S * vMag;
    dragX = -dragCoeff * vx;
    dragZ = -dragCoeff * vz;

    % 並進加速度
    ax = (thrust * eTx + dragX) / mTot;
    az = (thrust * eTz + dragZ) / mTot - cfgSI.g;

    dy = [vx; vz; ax; az; dVw; dMa];
end
