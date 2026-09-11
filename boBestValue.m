function b = boBestValue(Y, mode)
%BOBESTVALUE  Best (min or max) finite value in Y.
    Yf = Y(isfinite(Y));
    if isempty(Yf)
        b = NaN;
        return;
    end
    if mode == 'maximize'
        b = max(Yf);
    else
        b = min(Yf);
    end
end
