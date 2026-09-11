function [x, f] = boLbfgs(fun, x0, tol, maxIter, m)
%BOLBFGS  Compact L-BFGS for smooth, unconstrained scalar minimisation.
%
%   [x, f] = boLBFGS(fun, x0, tol, maxIter, m)
%
%   Minimises fun(x) (scalar) from start x0 (row vector) using a two-loop
%   L-BFGS recursion with an Armijo line search.  Finite-difference
%   gradients.  Used for the low-dimensional smooth sub-problems inside
%   bayes_optimize (GP hyper-parameters, acquisition maximisation).
%   Base MATLAB only.
%
%   All internal vectors are column vectors (n-by-1).

    if nargin < 3 || isempty(tol);     tol = 1e-6; end
    if nargin < 4 || isempty(maxIter); maxIter = 200; end
    if nargin < 5 || isempty(m);       m = 20; end

    x  = x0(:);            % column vector
    f0 = fun(x');          % fun expects a row vector
    if ~isfinite(f0)
        x = x'; f = f0;
        return;
    end
    g = boFiniteDiffGrad(fun, x', f0);   % row vector (1-by-n)
    g = g(:);                              % convert to column
    if norm(g) < tol
        x = x'; f = f0;
        return;
    end

    % L-BFGS memory (all column vectors).
    sQ = {}; yQ = {}; rhoQ = {};

    for it = 1:maxIter
        % ---- Two-loop recursion (all column vectors) ----------------------
        r = -g;
        aVals = zeros(numel(sQ),1);
        bVals = zeros(numel(sQ),1);

        % backward loop (oldest -> newest)
        for k = numel(sQ):-1:1
            aVals(k) = rhoQ{k} * (sQ{k}' * r);   % scalar
            r = r - aVals(k) * yQ{k};            % column
        end
        % forward loop (newest -> oldest)
        for k = 1:numel(sQ)
            bVals(k) = rhoQ{k} * (yQ{k}' * r);   % scalar
            r = r + (aVals(k) - bVals(k)) * sQ{k}; % column
        end
        d = r;   % column

        % ---- Ensure a descent direction -----------------------------------
        if d' * g > 0
            d = -g;
        end

        % ---- Armijo line search -------------------------------------------
        step = 1; c1 = 1e-4;
        dRow = d';   % row for fun()
        fT = fun(x' + step * dRow);
        ok = (fT < f0 + c1 * step * (g' * d));
        for lsIt = 1:30
            if ok, break; end
            step = step * 0.5;
            fT = fun(x' + step * dRow);
            ok = (fT < f0 + c1 * step * (g' * d));
        end
        if ~isfinite(fT)
            x = x'; f = f0;
            break;
        end
        xNew = x + step * d;
        gNew = boFiniteDiffGrad(fun, xNew', fT);   % row
        gNew = gNew(:);                             % column

        % ---- Update memory (curvature condition, FIFO) --------------------
        sNew = xNew - x;
        sy = sNew' * gNew;
        if sy > 1e-12
            sQ{end+1}   = (xNew - x);
            yQ{end+1}   = (gNew - g);
            rhoQ{end+1} = 1 / (sQ{end}' * yQ{end});
            if numel(sQ) > m
                sQ = sQ(2:end); yQ = yQ(2:end); rhoQ = rhoQ(2:end);
            end
        end

        x = xNew; f0 = fT; g = gNew;
        if norm(g) < tol
            break;
        end
    end
    f = f0;
    x = x';   % return as row vector
end
