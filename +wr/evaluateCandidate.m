function [cost, sim] = evaluateCandidate(q, cfgSI)
%EVALUATECANDIDATE Evaluate direct single-shooting objective J(q) = -Hmax(q).
%   [cost, sim] = wr.evaluateCandidate(q, cfgSI) simulates a candidate
%   decision vector q = [f, alpha_1, ..., alpha_N]. If flight succeeds and
%   reaches a valid positive apex, cost = -sim.Hmax. Otherwise returns a
%   finite positive penalty cost (never NaN) and retains diagnostic status
%   in sim.

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
        % Finite penalty > 0 so invalid/non-liftoff candidates are never chosen
        partialZ = 0.0;
        if ~isempty(sim.z) && all(isfinite(sim.z))
            partialZ = max(sim.z);
        end
        cost = 1.0e5 - min(max(partialZ, 0.0), 1.0e4);
    end
end
