function [y, nBad] = boEvalObjective(fun, X, opt)
%BOEVALOBJECTIVE  Evaluate the objective at every row of X.
%
%   If the user's function is vectorised (returns a vector when given a
%   matrix), it is called once.  Otherwise it is called row-by-row.
%   Non-finite results are replaced with NaN and counted in nBad.

    m = size(X,1);
    if iscell(fun)
        fn = fun{1}; x0 = fun{2};
        f = fn(X, x0);
    else
        f = fun(X);
    end
    f = f(:);
    % The vectorised call is only trusted if it returned EXACTLY one value per
    % point.  A size mismatch is the classic "not vectorised" case, but an
    % element-wise objective can also return the right shape while still
    % containing NaN/Inf in some rows (e.g. exp(...) overflow); in that case
    % the per-row evaluation below is the only safe way to isolate the bad
    % points, so fall back whenever the vectorised result is not exactly
    % m finite scalars.
    if numel(f) ~= m || ~all(isfinite(f))
        f = zeros(m,1);
        for i = 1:m
            if iscell(fun)
                f(i) = fun{1}(X(i,:), fun{2});
            else
                f(i) = fun(X(i,:));
            end
        end
    end
    nBad = ~isfinite(f);
    f(nBad) = NaN;
    y = f;
end
