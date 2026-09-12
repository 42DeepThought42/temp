function xNext = boMaximiseAcquisition(X, y, theta, opt, theta0)
%BOMAXIMISEACQUISITION  Maximise the acquisition function over [0,1]^d.
%
%   Uses random starts + Nelder-Mead local refinement.  Robust to the
%   non-smoothness of the acquisition function.

    d = size(X,2);
    gp = opt.gp;
    if ~isfield(gp, 'kernel') || isempty(gp.kernel)
        gp.kernel = 'matern32';
    end

    sigma2 = exp(2*theta(d+1));
    noise2 = exp(2*theta(d+2));
    ls     = exp(theta(1:d));

    % ---- One-time GP factorisation over the training data -----------------
    K  = boKernel(X, X, ls, sigma2, gp.kernel) + noise2*eye(size(X,1));
    K  = K + 1e-10*eye(size(K));
    try
        L  = chol(K, 'lower');
    catch
        xNext = rand(1, d);
        return;
    end
    w = L \ y;
    alpha = L' \ w;

    % ---- Acquisition as a scalar function of a row vector z ---------------
    acqNeg = @(z) -boAcqValue(z, X, y, alpha, L, ls, sigma2, gp.kernel, opt);

    % ---- Build start points -------------------------------------------------
    % Re-seed from the user-supplied seed (modern MATLAB convention) so that
    % the multistart locations are reproducible.  A seed of 0 means "do not
    % reset", in which case we continue the existing RNG stream.
    nRandom = max(50, 20 * d);
    if opt.seed > 0
        rng(opt.seed);
    end
    starts = rand(nRandom, d);
    bestRow = X(boBestIndex(y, opt.mode), :);
    starts  = [starts; bestRow];

    % ---- Nelder-Mead refinement from top starts -----------------------------
    nStarts = min(opt.nStarts, size(starts,1));

    acqVals = zeros(size(starts,1), 1);
    for i = 1:size(starts,1)
        acqVals(i) = acqNeg(starts(i,:));
    end
    [~, ord] = sort(acqVals);
    starts = starts(ord(1:nStarts), :);

    bestVal = inf;
    bestZ   = starts(1,:);
    for i = 1:nStarts
        z0 = starts(i,:);
        [z1, v1] = boNelderMead(acqNeg, z0, d, 1e-8, 200);
        if v1 < bestVal
            bestVal = v1;
            bestZ = z1;
        end
    end

    xNext = bestZ;
end
