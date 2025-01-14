import sympy as sp

def find_null(M: sp.Matrix) -> list[sp.Matrix]:
    """
    Computes and displays the null space (kernel) of a matrix
    
    @param M: The matrix
    """

    null_space = M.nullspace()
    
    print("Nullspace of the matrix:")
    if not null_space:
        print("The nullspace is trivial (only the zero vector).")
        return null_space

    for i, vec in enumerate(null_space, start=1):
        simplified_vec = sp.simplify(vec)
        print(f"Basis vector n°{i}:")
        sp.pprint(simplified_vec)
