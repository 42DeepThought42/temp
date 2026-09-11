function tf = boIsBetter(a, b, mode)
%BOISBETTER  True if `a` is a better objective value than `b`.
    if mode == 'maximize'
        tf = a > b;
    else
        tf = a < b;
    end
end
