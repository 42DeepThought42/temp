function p = boProbabilityImproveMax(mu, s, best, xi)
%BOPROBABILITYIMPROVEMAX  Probability of improvement for maximisation.
    z = (best + xi - mu) / max(s, 1e-12);
    p = boNormalCdf(z);
end
