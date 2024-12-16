function res = find_stationary_points(func, variable)
    func_dot = diff(func, variable);
    eq = func_dot == 0;
    res = solve(eq, [variable]);%, 'ReturnConditions',true);
end