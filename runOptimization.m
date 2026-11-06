function [result, figs] = runOptimization(cfg)
%RUNOPTIMIZATION Validate configuration, run water rocket optimization, and plot.
%   [result, figs] = runOptimization() uses the demo configuration from
%   wr.defaultConfig(), runs constrained optimization of initial water fill
%   fraction and variable nozzle schedule, prints a structured summary report,
%   and generates verification plots.
%
%   [result, figs] = runOptimization(cfg) runs with a user-supplied cfg struct.

    if nargin < 1 || isempty(cfg)
        cfg = wr.defaultConfig();
    end

    cfgSI = wr.validateConfig(cfg);
    result = wr.optimizeRocket(cfgSI);
    figs = wr.plotResults(result, cfgSI);

    printSummaryReport(result, cfgSI);
end

function printSummaryReport(result, cfgSI)
    fprintf('\n====================================================================\n');
    fprintf('  Water Rocket Variable-Nozzle Altitude Optimization Summary\n');
    fprintf('====================================================================\n');
    if isfield(cfgSI, 'isDemo') && cfgSI.isDemo
        fprintf('  [NOTE] Running with DEMO template parameters (not hardware-calibrated).\n');
    end
    fprintf('  Tank volume Vb        : %.3f L (%.6f m^3)\n', cfgSI.Vb * 1e3, cfgSI.Vb);
    fprintf('  Dry mass mDry         : %.3f kg\n', cfgSI.mDry);
    fprintf('  Body diameter D       : %.1f mm (%.4f m)\n', cfgSI.D * 1e3, cfgSI.D);
    fprintf('  Initial pressure      : %.1f kPaG (%.1f kPa abs)\n', ...
        cfgSI.pGauge * 1e-3, cfgSI.p0Abs * 1e-3);
    fprintf('  Launch angle theta0   : %.1f deg (%.4f rad)\n', ...
        rad2deg(cfgSI.theta0), cfgSI.theta0);
    fprintf('  Nozzle area bounds    : [%.3e, %.3e] m^2 (dEq: [%.2f, %.2f] mm)\n', ...
        cfgSI.Amin, cfgSI.Amax, ...
        sqrt(4 * cfgSI.Amin / pi) * 1e3, sqrt(4 * cfgSI.Amax / pi) * 1e3);
    fprintf('  Max area slew rate    : %.3e m^2/s\n', cfgSI.ARateMax);
    fprintf('  Control knots / end   : %d knots over [0, %.3f] s\n', ...
        cfgSI.nControlKnots, cfgSI.tControlEnd);
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Baseline (best const) : Hmax = %.3f m (f = %.3f, Vw0 = %.3f L, alpha = %.3f)\n', ...
        result.baseline.Hmax, result.baseline.f, result.baseline.Vw0_L, result.baseline.alpha(1));
    fprintf('  Optimal (variable)    : Hmax = %.3f m (f* = %.3f, Vw0* = %.3f L, tApex = %.3f s)\n', ...
        result.best.Hmax, result.best.f, result.best.Vw0_L, result.best.tApex);
    if isfinite(result.comparison.improvementPct)
        fprintf('  Altitude improvement  : +%.3f m (+%.2f %%)\n', ...
            result.comparison.improvementAbs, result.comparison.improvementPct);
    else
        fprintf('  Altitude improvement  : %.3f m (percentage N/A: non-positive baseline)\n', ...
            result.comparison.improvementAbs);
    end
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Optimal Knot Schedule :\n');
    for k = 1:numel(result.best.tKnots)
        fprintf('    k=%d: t=%.3f s | alpha=%.4f | A=%.4e m^2 | dEq=%.2f mm\n', ...
            k, result.best.tKnots(k), result.best.alpha(k), ...
            result.best.AKnots(k), result.best.dEqKnots(k) * 1e3);
    end
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Phase Events          :\n');
    fprintf('    Water phase end     : t = %.4f s (%s, Vw_rem = %.4e m^3)\n', ...
        result.phaseEvents.waterEnd.t, result.phaseEvents.waterEnd.reason, ...
        result.phaseEvents.waterEnd.VwRemaining);
    fprintf('    Air phase end       : t = %.4f s (%s, ma_rem = %.4e kg)\n', ...
        result.phaseEvents.airEnd.t, result.phaseEvents.airEnd.reason, ...
        result.phaseEvents.airEnd.maRemaining);
    fprintf('    Apex reached        : t = %.4f s (z = %.3f m)\n', ...
        result.phaseEvents.apex.t, result.phaseEvents.apex.z);
    if result.diagnostics.controlHoldActive
        fprintf('  [DIAGNOSTIC] Propulsion lasted until t=%.4f s (> tControlEnd=%.4f s);\n', ...
            result.diagnostics.tPropulsionEnd, cfgSI.tControlEnd);
        fprintf('               final knot opening was held after tControlEnd.\n');
    end
    fprintf('  Diagnostics           : solver=%s, exitflag=%d, maxViol=%.2e, maxSlew=%.3e m^2/s\n', ...
        result.diagnostics.solverUsed, result.diagnostics.exitflag, ...
        result.diagnostics.maxConstraintViolation, result.diagnostics.maxSlewRate);
    fprintf('                          ODE evals=%d, ODE failures=%d\n', ...
        result.diagnostics.nOdeEvaluations, result.diagnostics.nOdeFailures);
    fprintf('--------------------------------------------------------------------\n');
    fprintf('  Hardware & Safety Caveats:\n');
    for i = 1:numel(result.diagnostics.limitationsNote)
        fprintf('    %s\n', result.diagnostics.limitationsNote(i));
    end
    fprintf('====================================================================\n\n');
end
