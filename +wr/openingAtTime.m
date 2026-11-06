function [A, dEq] = openingAtTime(t, alpha, cfgSI)
%OPENINGATTIME 時刻 t におけるノズル有効開口面積 A と等価直径 dEq を計算する
%   区間 [0, tControlEnd] では等間隔ノット alpha の区分線形補間を行い、
%   区間外では端点の開度を保持する。

    N = cfgSI.nControlKnots;
    if ~isnumeric(alpha) || numel(alpha) ~= N || any(~isfinite(alpha(:)))
        error('wr:invalidNozzle', ...
            'Control vector alpha must be a finite numeric vector of length %d.', N);
    end

    alphaCol = alpha(:);
    if any(alphaCol < -1e-6 | alphaCol > 1 + 1e-6)
        error('wr:invalidNozzle', 'Control knot values alpha must lie in [0, 1].');
    end
    % ソルバの微小な数値誤差を [0, 1] にクリップ
    alphaCol = min(max(alphaCol, 0.0), 1.0);

    % 領域外は端点の時刻にクランプして値を保持
    tClamped = min(max(t, 0.0), cfgSI.tControlEnd);

    % 等間隔グリッド上の区分線形補間
    dt = cfgSI.tControlEnd / (N - 1);
    u = tClamped ./ dt;
    idx = min(floor(u) + 1, N - 1);
    frac = u - (idx - 1);

    alphaInterp = reshape(alphaCol(idx) + frac(:) .* (alphaCol(idx + 1) - alphaCol(idx)), size(t));
    alphaInterp = min(max(alphaInterp, 0.0), 1.0);

    A = cfgSI.Amin + (cfgSI.Amax - cfgSI.Amin) .* alphaInterp;
    if nargout > 1
        dEq = sqrt((4.0 / pi) .* A);
    end
end
