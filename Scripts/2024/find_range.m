function [J] = find_range(J)
        % Compute the range of the jacobian
        Range = orth(sym(J));

        % Display the range
        disp('range of the jacobian');
        disp('warning! this result might be wrong, please check by hand if possible')
        R=simplify(Range);
        [n,m]=size(R);
        for i=1:m
            [numerator, denominator] = numden(R(:,i));
            fprintf('%d basis',i)
            display(simplify(numerator,'Steps',50));
        end
end