function plot_function_check_start_end(func, variable, interval)
    if length(func) > 1
        hold off
        leg = {};
        for i=1:length(func)
            f(variable) = func(i);
            fplot(@(variable) f(variable), interval);
            
            leg{i} = int2str(i);
            hold on
        end
        legend(leg,'Location','southwest')
        hold off
    else
       f(variable) = func;
       fplot(@(variable) f(variable), interval);
    end
    
    for i=1:length(func)
        fprintf("----------------------------------\n");
        fprintf("START AT:\n");
        f = subs(func(i), {variable}, {interval(1)});
        display(eval(f));

        fprintf("END AT:\n");
        f = subs(func(i), {variable}, {interval(2)});
        display(eval(f));
        fprintf("----------------------------------\n");
    end

    
end