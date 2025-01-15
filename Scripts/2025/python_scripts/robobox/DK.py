import sympy as sp 
from .transformation_matrix import transformation_matrix

def DK(DH_table: list, x: int=0, y: int=None) -> dict:
    """
    Computes the Direct Kinematics of a serial robot

    @param DH_table: The Denavit-Hartenberg table
    @param x: Index of the starting joint (inclusive)
    @param y: Index of the ending joint (inclusive)
    """
    if y is None:
        y = len(DH_table)-1

    if x < 0 or y >= len(DH_table) or x > y:
        raise ValueError("Invalid joint indexes. Be sure that 0 <= x <= y < no. of joints.")

    Txy = transformation_matrix(DH_table, x, y)
    Txy = sp.simplify(Txy)

    # Position vector
    p = Txy[:3, 3]

    # Orientation axes
    xN = Txy[:3, 0]
    yN = Txy[:3, 1]
    zN = Txy[:3, 2]

    return {
        "Txy": Txy,
        "position": p,
        "x-axis": xN,
        "y-axis": yN,
        "z-axis": zN,
    }
