function plot_function(func, variable, interval)
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
end