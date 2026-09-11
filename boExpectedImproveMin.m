function ei = boExpectedImproveMin(mu, s, best, xi)
%BOEXPECTEDIMPROVEMIN  Expected improvement for minimisation.
    e = best - xi - mu;
    if s < 1e-12
        ei = max(e, 0);
    else
        z = e / s;
        ei = e * boNormalCdf(z) + s * boNormalPdf(z);
    end
end
