function M = T_to_M(T, q_dot)
    r = length(q_dot);
    M = sym(zeros(r));
    c = r;
    for i=1:r
        for j=1:c
            tmp = simplify(diff(T, q_dot(i)));
            M(i, j) = simplify(diff(tmp, q_dot(j)));
        end
    end
end