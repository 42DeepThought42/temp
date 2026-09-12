function X = boLatinHypercube(n, d, seed)
%BOLATINHIERCUBE  Space-filling Latin-hypercube sample in the unit cube.
%
%   X = boLATINHIERCUBE(n, d, seed)
%
%   Returns an n-by-d matrix of points in [0,1]^d.  One stratified random
%   point per row in each dimension.  Base MATLAB only.
%
%   Seeding follows the modern MATLAB convention: `rng(seed)` resets the
%   default (twister) stream, so the design is reproducible for a given
%   seed.  A seed of 0 (or a missing/empty seed) leaves the RNG state
%   untouched, i.e. no reseeding happens.

    if nargin >= 3 && ~isempty(seed) && seed > 0
        rng(seed);
    end
    X = zeros(n, d);
    for j = 1:d
        perm = randperm(n);
        X(:,j) = (perm - 0.5) / n;
    end
end
