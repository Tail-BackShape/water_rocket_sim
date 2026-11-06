function dy = rocketRHS(t, y, phase, alpha, cfgSI, phase0)
%ROCKETRHS Right-hand side of 2D point-mass equations of motion for water rocket.
%   dy = wr.rocketRHS(t, y, phase, alpha, cfgSI, phase0) computes the time
%   derivative dy = [vx; vz; ax; az; dVw; dMa] for state vector
%   y = [x; z; vx; vz; Vw; ma] in phase "water", "air", or "coast".

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

    % Total instantaneous mass
    mTot = cfgSI.mDry + cfgSI.rhoW * Vw + ma;

    % Velocity magnitude and thrust direction
    vMag = hypot(vx, vz);
    if vMag < cfgSI.vEpsilon
        eTx = cos(cfgSI.theta0);
        eTz = sin(cfgSI.theta0);
    else
        eTx = vx / vMag;
        eTz = vz / vMag;
    end

    % Aerodynamic drag force components
    S = pi * (cfgSI.D ^ 2) / 4.0;
    dragCoeff = 0.5 * cfgSI.rhoAir * cfgSI.Cd * S * vMag;
    dragX = -dragCoeff * vx;
    dragZ = -dragCoeff * vz;

    % Accelerations
    ax = (thrust * eTx + dragX) / mTot;
    az = (thrust * eTz + dragZ) / mTot - cfgSI.g;

    dy = [vx; vz; ax; az; dVw; dMa];
end
