function a = boAcqValue(z, X, y, alpha, L, ls, sigma2, kernel, opt)
%BOACQVALUE  Acquisition value at a single point z (row vector).
%
%   a = boACQVALUE(z, X, y, alpha, L, ls, sigma2, kernel, opt)
%
%   z:     1-by-d query point (unit cube)
%   X:     n-by-d training points
%   y:     n-by-1 training values (normalised)
%   alpha: n-by-1, K\y precomputed
%   L:     lower Cholesky factor of K + noise*I
%   ls:    1-by-d lengthscales
%   sigma2: signal variance
%   kernel: kernel name
%   opt:    options struct (mode, acquisition, xi, kappa)

    Kn  = boKernel(z(1,:), X, ls, sigma2, kernel);       % 1-by-n cross cov
    Kzz = boKernel(z(1,:), z(1,:), ls, sigma2, kernel);  % scalar
    mu  = Kn * alpha;
    w   = L \ Kn';           % n-by-1 (solve L*w = Kn')
    v   = L' \ w;            % n-by-1 (solve L'*v = w)
    s2  = max(Kzz - Kn * v, 0);  % scalar
    s   = sqrt(s2);

    switch upper(opt.acquisition)
        case 'EI'
            if opt.mode == 'maximize'
                a = boExpectedImproveMax(mu, s, max(y), opt.xi);
            else
                a = boExpectedImproveMin(mu, s, min(y), opt.xi);
            end
        case 'PI'
            if opt.mode == 'maximize'
                a = boProbabilityImproveMax(mu, s, max(y), opt.xi);
            else
                a = boProbabilityImproveMin(mu, s, min(y), opt.xi);
            end
        case 'UCB'
            a = mu + opt.kappa * s;
        case 'LCB'
            a = mu - opt.kappa * s;
        case 'ENTROPY'
            a = s;
        otherwise
            error('boAcqValue:unknown', 'Unknown acquisition "%s".', opt.acquisition);
    end
end
