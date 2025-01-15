import sympy as sp

def find_range(M: sp.Matrix) -> list[sp.Matrix]:
    """
    Computes and displays the range (image) of a matrix

    @param J: The Jacobian matrix
    """
    range_space = M.columnspace()

    print("Warning! This result might be wrong, please check by hand if possible.")
    print("Range of the Jacobian:")
    for i, vec in enumerate(range_space, start=1):
        simplified_vec = sp.simplify(vec)
        print(f"Basis vector n°{i}:")
        sp.pprint(simplified_vec)