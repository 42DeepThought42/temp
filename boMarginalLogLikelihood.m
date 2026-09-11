function ll = boMarginalLogLikelihood(X, y, theta, gp)
%BOARGINALLOGLIKHOOD  GP marginal log-likelihood.
    d = size(X,2);
    n = size(X,1);
    y = y(:);
    theta = theta(:);
    if size(y,1) ~= n
        error('boMarginalLogLikelihood:dim', 'size X=[%d %d] but size y=[%d %d]', n, d, size(y,1), size(y,2));
    end
    sigma2 = exp(2*theta(d+1));
    noise2 = exp(2*theta(d+2));
    ls     = exp(theta(1:d));

    K = boKernel(X, X, ls, sigma2, gp.kernel) + noise2*eye(n);
    K = K + 1e-10*eye(n);

    try
        L = chol(K, 'lower');
    catch
        ll = -inf;
        return;
    end
    if any(isnan(diag(L))) || any(diag(L) <= 0)
        ll = -inf;
        return;
    end
    w = L \ y;
    alpha = L' \ w;
    ll = -0.5 * y' * alpha - sum(log(diag(L))) - 0.5 * n * log(2*pi);
    % Weak prior: penalise extremely small noise variance to prevent degenerate fits
    ll = ll - 2 * max(0, log(1e-3) - theta(d+2));
    if ~isfinite(ll)
        ll = -inf;
    end
end
