function C = M_to_C(M, q, q_dot)
    C = sym([]);
    n = length(q);
    
    for i=1:n
        M_k = M(:, i);
        c_k = 0.5*(jacobian(M_k, q)+(jacobian(M_k, q)') - diff(M, q(i)));
        c_k = simplify(c_k);
        C = [C; simplify((q_dot')*c_k*q_dot)];
    end
end
