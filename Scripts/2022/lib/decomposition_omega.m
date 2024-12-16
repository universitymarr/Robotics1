function res = decomposition_omega(omega, angles_dot)
    res = sym(zeros(length(angles_dot)));
    for i=1:length(angles_dot)
        for j=1:length(angles_dot)
            res(i,j) = diff(omega(i), angles_dot(j));
        end
    end
end
