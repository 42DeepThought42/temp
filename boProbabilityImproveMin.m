function p = boProbabilityImproveMin(mu, s, best, xi)
%BOPROBABILITYIMPROVEMIN  Probability of improvement for minimisation.
    z = (best - xi - mu) / max(s, 1e-12);
    p = 1 - boNormalCdf(z);
end
