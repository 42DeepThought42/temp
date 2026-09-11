function p = boNormalPdf(z)
%BONORMALPDF  Standard normal PDF (base MATLAB).

    p = exp(-0.5 * z.^2) / sqrt(2*pi);
end