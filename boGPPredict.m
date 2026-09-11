function [mu, s2] = boGPPredict(X, y, theta, Xnew, gp)
%BOGPREDICT  Posterior mean and variance of a GP at Xnew given training data.
%
%   [mu, s2] = boGPREDICT(X, y, theta, Xnew, gp)
%
%   X:    n-by-d training points (unit cube)
%   y:    n-by-1 training values (normalised)
%   theta: (d+2)-by-1 hyper-parameters
%   Xnew: m-by-d query points
%   gp:   struct with .kernel field

    d = size(X,2);
    y = y(:);
    theta = theta(:);
    if ~isfield(gp, 'kernel') || isempty(gp.kernel)
        gp.kernel = 'matern32';
    end
    sigma2 = exp(2*theta(d+1));
    noise2 = exp(2*theta(d+2));
    ls     = exp(theta(1:d));
    n = size(X,1);
    m = size(Xnew,1);

    K   = boKernel(X, X,       ls, sigma2, gp.kernel) + noise2*eye(n);
    Kn  = boKernel(X, Xnew,    ls, sigma2, gp.kernel);   % n-by-m
    Knn = boKernel(Xnew, Xnew, ls, sigma2, gp.kernel);   % m-by-m

    K = K + 1e-10*eye(n);
    try
        L = chol(K, 'lower');
    catch
        mu = mean(y) * ones(m,1);
        s2 = sigma2 * ones(m,1);
        return;
    end
    % alpha = K \ y  (n-by-1)
    w = L \ y;
    alpha = L' \ w;
    % mu = Kn' * alpha  (m-by-1)
    mu = Kn' * alpha;
    % For variance: s2(i) = Knn(i,i) - Kn(:,i)' * K^{-1} * Kn(:,i)
    % Compute V = K^{-1} * Kn  (n-by-m)
    w2 = L \ Kn;
    V = L' \ w2;
    s2 = diag(Knn) - sum(Kn .* V, 1)';
    s2 = max(s2, 0);
end
