function [x, y] = find_abs_maximum_function_in_interval(func, variable, interval)
    stat = find_stationary_points(func, variable);
    array_peaks = {};
    
    x = -1;
    y = -1;
    
    %filter out stationary points outside interval
    stat_filtered = {};
    for i=1:length(stat)
        if stat(i) >= interval(1) && stat(i) <= interval(2)
            stat_filtered{end+1} = stat(i);
        end
    end
    
    for i=1:length(stat_filtered)
        array_peaks{i} = eval(abs(subs(func, {variable}, stat_filtered(i))));
        if array_peaks{i} > y
            y = array_peaks{i};
            x = stat_filtered{i};
        end
    end
end