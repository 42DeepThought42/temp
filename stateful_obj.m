function y = stateful_obj(x, x0)
%STATEFUL_OBJ  Example stateful objective: (x - x0)^2.
    y = (x(1) - x0)^2;
end
