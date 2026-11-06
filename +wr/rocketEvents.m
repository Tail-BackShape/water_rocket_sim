function [value, isterminal, direction] = rocketEvents(t, y, phase, alpha, cfgSI, phase0)
%ROCKETEVENTS 各飛行フェーズの ode45 イベント関数
%   Phase "water": [水枯渇 Vw=0; 圧力平衡 p=pAmb; 最高点 vz=0; 着地 z=0]
%   Phase "air"  : [圧力平衡 p=pAmb; 最高点 vz=0; 着地 z=0]
%   Phase "coast": [最高点 vz=0; 着地 z=0]

    z = y(2);
    vz = y(4);
    Vw = y(5);
    % 流出速度が sqrt(p - pAmb) に比例し p=pAmb で勾配ゼロとなるため、
    % 0.01 Pa の微小閾値で確実に符号反転を検出する
    pCutTol = 1e-2;

    switch lower(char(phase))
        case 'water'
            prop = wr.waterPropulsion(t, y, alpha, cfgSI);
            value = [ ...
                Vw; ...
                (prop.p - cfgSI.pAmb) - pCutTol; ...
                vz; ...
                z];
            isterminal = [1; 1; 1; 1];
            direction = [-1; -1; -1; -1];

        case 'air'
            prop = wr.airPropulsion(t, y, alpha, cfgSI, phase0);
            value = [ ...
                (prop.p - cfgSI.pAmb) - pCutTol; ...
                vz; ...
                z];
            isterminal = [1; 1; 1];
            direction = [-1; -1; -1];

        case 'coast'
            value = [ ...
                vz; ...
                z];
            isterminal = [1; 1];
            direction = [-1; -1];

        otherwise
            error('wr:invalidPhase', 'Unknown flight phase "%s".', char(phase));
    end
end
