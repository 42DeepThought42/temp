function g = boFiniteDiffGrad(fun, x, f0)
%BOFINITEDIFFGRAD  Central finite-difference gradient of a scalar function.
%
%   g = boFINITEDIFFGRAD(fun, x, f0)
%
%   fun: scalar function of a row vector
%   x:   1-by-n current point
%   f0:  fun(x) (precomputed to save one evaluation)
%   Returns 1-by-n gradient.

    n = numel(x);
    g = zeros(1, n);
    h = 1e-6;
    for j = 1:n
        e = zeros(1, n); e(j) = 1;
        fp = fun(x + h*e);
        fm = fun(x - h*e);
        if ~isfinite(fp); fp = f0; end
        if ~isfinite(fm); fm = f0; end
        g(j) = (fp - fm) / (2*h);
    end
    g(~isfinite(g)) = 0;
end
