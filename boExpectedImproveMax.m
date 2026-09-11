function ei = boExpectedImproveMax(mu, s, best, xi)
%BOEXPECTEDIMPROVEMAX  Expected improvement for maximisation.
    e = mu - best - xi;
    if s < 1e-12
        ei = max(e, 0);
    else
        z = e / s;
        ei = e * boNormalCdf(z) + s * boNormalPdf(z);
    end
end
