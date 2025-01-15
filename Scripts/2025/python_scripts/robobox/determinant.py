import sympy as sp

def determinant(M: sp.Matrix):
    """
    Computes the determinant of a matrix:

    @param M: The matrix
    """
    m, n = M.shape

    if m != n:  # Non-square case
        print("The matrix is not square:")
        if n - m >= 2:
            det_M = sp.simplify((M.T @ M).det())
            print("==== Determinant of (M.T * M) ====")
            sp.pprint(det_M)
        
        elif m < n:  # More columns than rows
            print("Removing one column at a time:")
            for i in range(n):
                Mcopy = M.copy()
                Mcopy.col_del(i)
                det_M = sp.simplify(Mcopy.det())
                print(f"==== Determinant of the submatrix with column n°{i+1} removed ====")
                sp.pprint(det_M)

        else:  # More rows than columns
            print("Removing rows to create square submatrices:")
            if m == 6 and n == 4:
                print("Special case: 6x4 matrix.")
                for i in range(m):
                    Mcopy = M.copy()
                    Mcopy.row_del(i)
                    for j in range(m - 1):
                        Mcopy2 = Mcopy.copy()
                        Mcopy2.row_del(j)
                        det_M = sp.simplify(Mcopy2.det())
                        print(f"==== Determinant of submatrix (row n°{i+1} and n°{j+2} removed) ====")
                        sp.pprint(det_M)
            else:
                det_M = sp.simplify((M.T * M).det())
                print("==== Determinant of (M.T * M) ====")
                sp.pprint(det_M)
    
    else:  # Square matrix
        det_M = sp.simplify(M.det())
        print("==== Determinant of the matrix ====")
        sp.pprint(det_M)   
    
    return det_M

