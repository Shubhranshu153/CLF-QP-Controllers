function [value,isterminal,direction] = compassEventsFcn(t,x)
    value = guard(x);
    isterminal = 1;
    direction = -1;
end

