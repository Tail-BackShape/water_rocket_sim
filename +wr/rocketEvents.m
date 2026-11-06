function [value, isterminal, direction] = rocketEvents(t, y, phase, alpha, cfgSI, phase0)
%ROCKETEVENTS ODE event function for water, air, and coast flight phases.
%   [value, isterminal, direction] = wr.rocketEvents(t, y, phase, alpha, cfgSI, phase0)
%   computes terminal event values for ode45:
%     Phase "water":
%       1. Water volume depleted (Vw == 0, decreasing)
%       2. Tank pressure equalized to ambient (p - pAmb == 0, decreasing)
%       3. Apex reached after liftoff (vz == 0, decreasing)
%       4. Ground impact (z == 0, decreasing)
%     Phase "air":
%       1. Tank pressure equalized to ambient (p - pAmb == 0, decreasing)
%       2. Apex reached (vz == 0, decreasing)
%       3. Ground impact (z == 0, decreasing)
%     Phase "coast":
%       1. Apex reached (vz == 0, decreasing)
%       2. Ground impact (z == 0, decreasing)

    z = y(2);
    vz = y(4);
    Vw = y(5);
    % Pressure equalization threshold [Pa]: because outflow velocity scales as
    % sqrt(p - pAmb), dp/dt -> 0 tangentially at p == pAmb and never crosses
    % below pAmb. A 0.01 Pa threshold (< 1e-7 * pAmb) ensures clean sign change.
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
