function K = boMaternKernel(X1, X2, ls, nu)
%BOMATERNKERNEL  Matérn kernel (nu = 0.5, 1.5 or 2.5).
%
%   K = boMATERNKERNEL(X1, X2, ls, nu)

    D = boScaledSqDist(X1, X2, ls);
    sqrt3 = sqrt(3);
    switch nu
        case 0.5
            K = exp(-sqrt3 .* sqrt(D));
        case 1.5
            r = sqrt3 .* sqrt(D);
            K = (1 + r) .* exp(-r);
        case 2.5
            r = sqrt3 .* sqrt(D);
            K = (1 + r + r.^2/3) .* exp(-r);
        otherwise
            error('boMaternKernel:nu', 'nu must be 0.5, 1.5 or 2.5.');
    end
end
