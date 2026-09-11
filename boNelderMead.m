function [x, f] = boNelderMead(fun, x0, d, tol, maxIter)
%BONELDERMEAD  Nelder-Mead simplex method (no gradients, bounded-safe).
%
%   [x, f] = boNELDERMEAD(fun, x0, d, tol, maxIter)
%
%   Robust local minimiser used as a final polish step after L-BFGS.  Base
%   MATLAB only.  Points are clipped to [0,1]^d internally so the simplex
%   cannot drift out of the feasible box.

    if nargin < 4 || isempty(tol);     tol = 1e-8; end
    if nargin < 5 || isempty(maxIter); maxIter = 100; end

    alpha = 1;   beta = 0.5; gamma = 2;
    x = x0(:)';
    f = fun(x);

    % Build initial simplex: unit vertex + unit-cube directions.
    simplex = [x];
    for j = 1:d
        v = x; v(j) = v(j) + 0.05;
        v = min(max(v,0),1);
        simplex = [simplex; v];
    end
    fs = zeros(size(simplex,1),1);
    for i = 1:size(simplex,1)
        fs(i) = fun(simplex(i,:));
    end

    for it = 1:maxIter
        % Order vertices by function value.
        [fs, ord] = sort(fs);
        simplex = simplex(ord,:);

        % Convergence on the vertex spread.
        if max(fs) - min(fs) < tol
            break;
        end

        centroid = mean(simplex(1:end-1,:), 1);
        centroid = min(max(centroid,0),1);

        % Reflection
        xR = centroid + alpha * (centroid - simplex(end,:));
        xR = min(max(xR,0),1);
        fR = fun(xR);

        if fR < fs(1)
            % Expansion
            xE = centroid + gamma * (xR - centroid);
            xE = min(max(xE,0),1);
            fE = fun(xE);
            if fE < fR
                simplex(end,:) = xE; fs(end) = fE;
            else
                simplex(end,:) = xR; fs(end) = fR;
            end
        elseif fR < fs(end-1)
            simplex(end,:) = xR; fs(end) = fR;
        else
            % Contraction
            xC = centroid + beta * (simplex(end,:) - centroid);
            xC = min(max(xC,0),1);
            fC = fun(xC);
            if fC < fs(end)
                simplex(end,:) = xC; fs(end) = fC;
            else
                % Shrink
                for i = 2:size(simplex,1)
                    simplex(i,:) = simplex(1,:) + 0.5 * (simplex(i,:) - simplex(1,:));
                    fs(i) = fun(simplex(i,:));
                end
            end
        end
    end

    [~, bi] = min(fs);
    x = simplex(bi,:);
    f = fs(bi);
end