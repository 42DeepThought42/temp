function X = boDenormalise(Xn, bounds)
%BOdenormalise  Map points from the unit hyper-cube back to the original box.
%
%   X = boDENORMALISE(Xn, bounds)
%
%   Xn: m-by-d in [0,1]^d, bounds: 2-by-d.  Returns m-by-d in the original box.

    lo = bounds(1,:); hi = bounds(2,:);
    X = Xn .* (hi - lo) + lo;
end