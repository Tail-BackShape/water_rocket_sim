function [cost, sim] = evaluateCandidate(q, cfgSI)
%EVALUATECANDIDATE 決定変数 q = [f, alpha_1, ..., alpha_N] の目的関数 J = -Hmax を評価する
%   飛行成功時は cost = -sim.Hmax、飛行不能・失敗時は正の有限ペナルティ値を返す。

    expectedLen = 1 + cfgSI.nControlKnots;
    if ~isnumeric(q) || numel(q) ~= expectedLen || any(~isfinite(q(:)))
        sim = wr.simulateRocket(NaN, nan(1, cfgSI.nControlKnots), cfgSI);
        sim.status = "invalidInput";
        sim.message = sprintf('Decision vector q must be a finite numeric vector of length %d.', expectedLen);
        cost = 1.0e6;
        return;
    end

    qRow = q(:).';
    f = qRow(1);
    alpha = qRow(2:end);

    sim = wr.simulateRocket(f, alpha, cfgSI);

    if string(sim.status) == "success" && isfinite(sim.Hmax) && sim.Hmax > 0.0
        cost = -sim.Hmax;
    else
        % 無効・離陸不能な候補が採用されないよう正の有限ペナルティを返す
        partialZ = 0.0;
        if ~isempty(sim.z) && all(isfinite(sim.z))
            partialZ = max(sim.z);
        end
        cost = 1.0e5 - min(max(partialZ, 0.0), 1.0e4);
    end
end
