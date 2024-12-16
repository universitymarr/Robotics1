function [x, y] = find_sym_abs_max_in_positive_interval(func, variable, variable_time, max_interval)
    %interval must be [0, k]
    CONST = 1; % must be 1
    func_eval = subs(func, {variable}, {CONST}); %function with variable fixed (T=1)
    limit_low = 0;
    limit_high = subs(max_interval, {variable}, {CONST});
    
    if is_line_or_poly(func, variable_time)
        % is a line
        sx = subs(func_eval, {variable_time}, {limit_low});
        dx = subs(func_eval, {variable_time}, {limit_high});
        
        if sx > dx
            x = 0;
            y = subs(func, {variable_time}, {x});
        else
            x = max_interval;
            y = subs(func, {variable_time}, {x});
        end
    else
        % is a poly
        
        % get numerical value
        inter = [limit_low, limit_high];
        [x_eval, y_eval] = find_abs_maximum_function_in_interval(func_eval, variable_time, inter);
        
        %rewrite x in the symbolic interval
        x = x_eval * max_interval;
        y = subs(func, {variable_time}, {x});
    end
end

function bool = is_line_or_poly(func, variable)
    % func must be a line or a polynomial !
    % return true if is a line
    f_diff = diff(func, variable);
    
    f1 = subs(f_diff, {variable}, {1000});
    
    bool = false;
    if f_diff == f1
        bool = true;
    end
end

