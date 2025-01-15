import sympy as sp 
from .find_null import find_null
from .find_range import find_range
from .determinant import determinant

def jacobian_analysis(J: sp.Matrix, variables: list[sp.Symbol]):
    """
    Performs a full analysis of the provided Jacobian matrix:
    - Determinant
    - Singularities
    - Rank 
    - Nullspace
    - Range
    - Complementary Nullspace

    @param J: The Jacobian Matrix
    @param variables: The variables list
    """
    print("==== Jacobian Analysis ====")
    sp.pprint(J)

    m, n = J.shape

    print(f"Shape of the Jacobian is {m}x{n}")
    if m != n:
        print("The Jacobian is not square")
        det_J = determinant(J.T@J)
        print("\n==== Singularity conditions ====")
        singularity = sp.solve(det_J, variables)
        sp.pprint(singularity)

    else:
        print("The Jacobian is square")
        det_J = determinant(J)
        print("\n==== Singularity conditions ====")
        singularity = sp.solve(det_J, variables)
        sp.pprint(singularity)

    print("\n==== Rank of the Jacobian ====")
    rank_J = J.rank()
    print(f"Rank: {rank_J}")
    
    print("\n==== Range of the Jacobian ====")
    find_range(J)

    print("\n==== Nullspace of the Jacobian ====")
    find_null(J)

    print("\n==== Complementary nullspace of the Jacobian ====")
    complementary_nullspace = J.T.nullspace()
    if not complementary_nullspace:
        print("The complementary nullspace is only the 0 vector")
    else:
        for i, vec in enumerate(complementary_nullspace, start=1):
            simplified_vec = sp.simplify(vec)
            print(f"Basis vector n°{i}:")
            sp.pprint(simplified_vec)

    print("\n==== Dimension checking ====")
    print("dim Range(J) + dim Null(J^T):", len(J.columnspace()) + len(complementary_nullspace))
    print("..should equal the task-space dimension (rows of the Jacobian).")
    print("dim Range(J^T) + dim Null(J):", len(J.T.columnspace()) + len(J.nullspace()))
    print("..should equal the joint-space dimension (columns of the Jacobian).")

