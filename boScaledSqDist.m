function D = boScaledSqDist(X1, X2, ls)
%BOSCALEDISQDIST  Squared distance scaled by per-dimension lengthscales.
%
%   D = boSCALEDISQDIST(X1, X2, ls)
%
%   X1: n-by-d, X2: m-by-d, ls: 1-by-d.  Returns n-by-m matrix of
%   (X1_i - X2_j)' diag(1/ls.^2) (X1_i - X2_j).

    D = zeros(size(X1,1), size(X2,1));
    for j = 1:numel(ls)
        D = D + (X1(:,j) - X2(:,j)').^2 / ls(j)^2;
    end
end
