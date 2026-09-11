function X = boLatinHypercube(n, d, seed)
%BOLATINHIERCUBE  Space-filling Latin-hypercube sample in the unit cube.
%
%   X = boLATINHIERCUBE(n, d, seed)
%
%   Returns an n-by-d matrix of points in [0,1]^d.  One stratified random
%   point per row in each dimension.  Base MATLAB only.

    if nargin >= 3 && ~isempty(seed)
        % Re-seed only if a positive integer seed is supplied.
        if seed > 0
            rand('seed', seed);
        end
    end
    X = zeros(n, d);
    for j = 1:d
        perm = randperm(n);
        X(:,j) = (perm - 0.5) / n;
    end
end