function K = boKernel(X1, X2, ls, sigma2, kernel)
%BOKERNEL  Separable kernel matrix between X1 (n-by-d) and X2 (m-by-d).
%
%   K = boKERNEL(X1, X2, ls, sigma2, kernel)
%
%   Supported kernels: 'matern52', 'matern32', 'matern12', 'rbf'.
%   ls is a row vector of per-dimension lengthscales.

    switch upper(char(kernel))
        case 'MATERN52'
            K = sigma2 * boMaternKernel(X1, X2, ls, 2.5);
        case 'MATERN32'
            K = sigma2 * boMaternKernel(X1, X2, ls, 1.5);
        case 'MATERN12'
            K = sigma2 * boMaternKernel(X1, X2, ls, 0.5);
        case 'RBF'
            K = sigma2 * exp(-0.5 * boScaledSqDist(X1, X2, ls));
        otherwise
            error('boKernel:unknown', 'Unknown kernel "%s".', kernel);
    end
end
