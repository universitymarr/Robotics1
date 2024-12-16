function S_s = M_to_S_skew_sym(M, q, q_dot, print_info)
    cell_c_k = christoffel_symbols(M, q);

    if print_info
        fprintf("----------------------------------------------------\n");
        fprintf("Matrices of Christoffel symbols (c_i):\n");
        print_cells(cell_c_k)
        fprintf("\n");
    end
    
    S_s = sym([]);
    
    n = length(q); 
    for i=1:n
        s_i = q_dot'*cell_c_k{i};
        S_s = [S_s; s_i];
    end
    
    if print_info
        fprintf("Row i-th of S matrix is given by s_i = q_dot' * c_i\n");
        fprintf("Result S:\n");
        display(S_s);
        fprintf("----------------------------------------------------\n");
    end
end
