function cell_c_k = christoffel_symbols(M, q)
    n = length(q);    
    cell_c_k = cell(1, n);
    
    for i=1:n
        M_k = M(:, i);
        c_k = 0.5*(jacobian(M_k, q)+(jacobian(M_k, q)') - diff(M, q(i)));
        c_k = simplify(c_k);
        cell_c_k{i} = c_k;
    end
end