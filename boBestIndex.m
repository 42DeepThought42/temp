function i = boBestIndex(Y, mode)
%BOBESTINDEX  Index of the best (min or max) value in Y.
    if mode == 'maximize'
        [~, i] = max(Y);
    else
        [~, i] = min(Y);
    end
end
