function Xn = boNormalise(X, bounds)
%BONORMALISE  Map points from the original box to the unit hyper-cube.
%
%   Xn = boNORMALISE(X, bounds)
%
%   X: m-by-d, bounds: 2-by-d.  Returns m-by-d in [0,1]^d.

    lo = bounds(1,:); hi = bounds(2,:);
    Xn = (X - lo) ./ (hi - lo);
end