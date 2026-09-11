function [theta, llh] = boFitGP(X, y, gp, theta0)
%BOFITGP  Fit GP hyper-parameters by maximising the marginal log-likelihood.
%
%   [theta, llh] = boFITGP(X, y, gp, theta0)
%
%   X: n-by-d training points (unit cube). y: n-by-1 (normalised).
%   theta parameterisation:
%       theta(1:d)   = log(lengthscales)     (one per dimension)
%       theta(d+1)   = log(signal variance)
%       theta(d+2)   = log(noise variance)
%   Returns the maximising theta and its marginal log-likelihood.

    d = size(X,2);
    y = y(:);

    if ~isfield(gp, 'kernel') || isempty(gp.kernel)
        gp.kernel = 'matern32';
    end
    tol   = 1e-6;
    if isfield(gp, 'tol') && ~isempty(gp.tol), tol = gp.tol; end
    maxIt = 200;
    if isfield(gp, 'maxIter') && ~isempty(gp.maxIter), maxIt = gp.maxIter; end

    theta = theta0(:);
    if isfield(gp, 'lengthscales') && ~isempty(gp.lengthscales)
        theta(1:d) = log(gp.lengthscales(:));
    end
    if isfield(gp, 'signalVar') && ~isempty(gp.signalVar)
        theta(d+1) = log(gp.signalVar);
    end
    if isfield(gp, 'noiseVar') && ~isempty(gp.noiseVar)
        theta(d+2) = log(gp.noiseVar);
    end

    if size(X,1) ~= size(y,1)
        error('boFitGP:dim', 'X is %s, y is %s', mat2str(size(X)), mat2str(size(y)));
    end
    negLLH = @(t) -boMarginalLogLikelihood(X, y, t, gp);
    [theta, negLL] = boLbfgs(negLLH, theta(:)', tol, maxIt, 30);
    theta = theta(:);
    llh = -negLL;

    % Clamp noise variance away from zero (prevent degenerate fits).
    theta(d+2) = max(theta(d+2), log(1e-4));
end
