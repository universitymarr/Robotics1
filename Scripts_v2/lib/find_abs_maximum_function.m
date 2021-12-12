function [x, y] = find_abs_maximum_function(func, variable)
    stat = find_stationary_points(func, variable);
    array_peaks = {};
    
    x = -1;
    y = -1;
    for i=1:length(stat)
        array_peaks{i} = eval(abs(subs(func, {variable}, stat(i))));
        if array_peaks{i} > y
            y = array_peaks{i};
            x = eval(stat(i));
        end
    end
end