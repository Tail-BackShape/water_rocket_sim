function [A, dEq] = openingAtTime(t, alpha, cfgSI)
%OPENINGATTIME Compute effective nozzle area and equivalent diameter at time t.
%   [A, dEq] = wr.openingAtTime(t, alpha, cfgSI) evaluates the piecewise
%   linear control schedule defined by normalized knot values alpha in [0, 1]
%   uniformly spaced over [0, cfgSI.tControlEnd]. For t < 0, the initial
%   value alpha(1) is held; for t > cfgSI.tControlEnd, the final value
%   alpha(end) is held.

    N = cfgSI.nControlKnots;
    if ~isnumeric(alpha) || numel(alpha) ~= N || any(~isfinite(alpha(:)))
        error('wr:invalidNozzle', ...
            'Control vector alpha must be a finite numeric vector of length %d.', N);
    end

    alphaCol = alpha(:);
    if any(alphaCol < -1e-6 | alphaCol > 1 + 1e-6)
        error('wr:invalidNozzle', 'Control knot values alpha must lie in [0, 1].');
    end
    % Clip tiny numerical solver tolerances strictly to [0, 1]
    alphaCol = min(max(alphaCol, 0.0), 1.0);

    % Clamp time to [0, tControlEnd] for endpoint hold
    tClamped = min(max(t, 0.0), cfgSI.tControlEnd);

    % Piecewise linear interpolation on uniform grid [0, tControlEnd]
    dt = cfgSI.tControlEnd / (N - 1);
    u = tClamped ./ dt;
    idx = min(floor(u) + 1, N - 1);
    frac = u - (idx - 1);

    alphaInterp = reshape(alphaCol(idx) + frac(:) .* (alphaCol(idx + 1) - alphaCol(idx)), size(t));
    % Guarantee no overshoot outside [0, 1]
    alphaInterp = min(max(alphaInterp, 0.0), 1.0);

    A = cfgSI.Amin + (cfgSI.Amax - cfgSI.Amin) .* alphaInterp;
    if nargout > 1
        dEq = sqrt((4.0 / pi) .* A);
    end
end
