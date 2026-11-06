function prop = waterPropulsion(t, y, alpha, cfgSI)
%WATERPROPULSION Compute thermodynamic state, mass flow, and thrust in Phase W.
%   prop = wr.waterPropulsion(t, y, alpha, cfgSI) evaluates the adiabatic
%   ideal-gas expansion of trapped air and Bernoulli water discharge at time t
%   for state vector y = [x; z; vx; vz; Vw; ma].

    % Clamp water volume within physical tank bounds to guard against tiny ODE
    % step overshoots while event detection locates Vw == 0
    VwRaw = y(5);
    Vw = min(max(VwRaw, 0.0), cfgSI.Vb * (1.0 - 1e-12));
    ma = max(y(6), 0.0);

    % Initial trapped air volume recovered from constant air mass ma = ma0
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
