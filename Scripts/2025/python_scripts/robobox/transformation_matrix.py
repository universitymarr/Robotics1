import sympy as sp
from .DH_matrix import DH_matrix

def transformation_matrix(DH_table: list, x: int, y: int) -> sp.Matrix:
    """
    Computes the transformation matrix ^xA_y from joint x to y

    @param DH_table: The Denavit-Hartenberg table
    @param x: Index of the starting joint (inclusive)
    @param y: Index of the ending joint (inclusive)
    """
    DH = DH_matrix()

    # Indexes assesrtions
    if x < 0 or y >= len(DH_table) or x > y:
        raise ValueError("Invalid joint indexes. Be sure that 0 <= x <= y < no. of joints.")
    
    T = sp.eye(4)

    # Computing the cumulative transformations
    for i in range(x, y+1):
        A_i = DH.subs({
            'alpha': DH_table[i][0],
            'a': DH_table[i][1],
            'd': DH_table[i][2],
            'theta': DH_table[i][3],
        })
    
        T = T @ A_i

    return T