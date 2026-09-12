function [fBest, xBest01, info] = boCoreLoop(fun, nvars, bounds, opt)
%BOCORELOOP  Main Bayesian-optimisation loop (works in the unit cube).
%
%   [fBest, xBest01, info] = boCoreLoop(fun, nvars, bounds, opt)
%
%   All points are kept in the unit hyper-cube [0,1]^d.  `fun` is evaluated
%   on the ORIGINAL scale (the caller denormalises before each evaluation).

    % ---- Initial design ------------------------------------------------------
    if opt.initialMethod == 'lhs'
        X0 = boLatinHypercube(opt.initialPoints, nvars, opt.seed);
    else
        X0 = rand(opt.initialPoints, nvars);
    end

    % ---- Append any user supplied fixed points -------------------------------
    if ~isempty(opt.fixedPoints)
        Xf = boNormalise(opt.fixedPoints, bounds);
        yf = opt.fixedValues(:);
        X0 = [X0; Xf];
        knownMask = [false(opt.initialPoints,1); true(size(yf,1),1)];
        yKnown    = [NaN(opt.initialPoints,1); yf];
    else
        knownMask = false(size(X0,1),1);
        yKnown    = NaN(size(X0,1),1);
    end

    % ---- Evaluate the initial design ----------------------------------------
    [y, ~] = boEvalObjective(fun, boDenormalise(X0, bounds), opt);
    y(knownMask) = yKnown(knownMask);
    X = X0;  Y = y(:);

    % ---- Bookkeeping ---------------------------------------------------------
    fBest   = boBestValue(Y, opt.mode);
    xBest01 = X(boBestIndex(Y, opt.mode), :);
    history = fBest;
    bestSoFar = fBest;

    yModel = Y;
    if opt.normalizeY
        yModel = (Y - min(Y)) / max(1, max(Y) - min(Y));
    end

    gp     = opt.gp;
    theta0 = [log(1) * ones(1, nvars), log(1.0), log(1e-4)];  % 1-by-(nvars+2)
    % Start with lengthscales ~ 1 (unit cube), signal var ~ 1, small noise.

    if opt.verbose
        fprintf('Bayesian optimisation: %s, d=%d, maxIter=%d\n', ...
                upper(opt.mode), nvars, opt.maxIter);
        fprintf('  it    best-value        improvement\n');
    end

    earlyStopEnabled = (opt.stopTolerance > 0);
    plateau = 0;
    for it = 1:opt.maxIter
        % 1) Fit the GP surrogate.
        [theta, ~] = boFitGP(X, yModel, gp, theta0);

        % 2) Maximise the acquisition function.
        xNext = boMaximiseAcquisition(X, yModel, theta, opt, theta0);

        % 3) Evaluate the true objective at xNext (original scale).
        [yNew, ~] = boEvalObjective(fun, boDenormalise(xNext(1,:), bounds), opt);

        % 4) Update the data set, then (re)build the normalised training values
        %    against the FULL, updated data so the GP always sees a consistent
        %    0-1 scale (re-normalising rather than appending keeps min/max
        %    aligned with the data actually being fitted).
        X = [X; xNext]; Y = [Y; yNew];
        if opt.normalizeY
            yModel = (Y - min(Y)) / max(1, max(Y) - min(Y));
        else
            yModel = Y;
        end
        bestIdx = boBestIndex(Y, opt.mode);
        curBest = Y(bestIdx);

        if boIsBetter(curBest, fBest, opt.mode)
            fBest = curBest; xBest01 = X(bestIdx,:);
            % Reset the patience counter whenever a *meaningful* new best is
            % found.  "Meaningful" is a relative improvement larger than
            % stopTolerance; the floor in the denominator avoids blow-up when
            % the best value is near zero (e.g. a benchmark minimum of 0).
            relImprove = abs(curBest - bestSoFar) / max(abs(bestSoFar), 1e-12);
            if relImprove < opt.stopTolerance
                plateau = plateau + 1;
            else
                plateau = 0;
            end
            bestSoFar = curBest;
        else
            plateau = plateau + 1;
        end

        history = [history, fBest];

        if opt.verbose
            fprintf('  %3d  %+.6e   %+.6e\n', it, fBest, fBest - history(end-1));
        end

        if earlyStopEnabled && plateau >= opt.plateauLength
            if opt.verbose
                fprintf('  early stop: no meaningful improvement for %d iters\n', ...
                        opt.plateauLength);
            end
            break;
        end
    end

    info = struct('X', X, 'F', Y, 'history', history, ...
                  'bestIter', boBestIndex(Y, opt.mode), 'nEvals', size(X,1));
end
