function p = boNormalCdf(z)
%BONORMALCDF  Standard normal CDF via the error function (base MATLAB).
%
%   p = boNORMALCDF(z)
%
%   p = Phi(z) = 0.5 * (1 + erf(z / sqrt(2))).

    p = 0.5 * (1 + erf(z / sqrt(2)));
end