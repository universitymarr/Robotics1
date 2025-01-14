import sympy as sp

def DH_matrix() -> sp.Matrix:
    """
    Returns the Denavit-Hartenberg matrix for a robot
    """
    # Parameters
    alpha, a, d, theta = sp.symbols("alpha a d theta")

    # DH transformation matrix
    DH_mat = sp.Matrix([
        [sp.cos(theta), -sp.cos(alpha)*sp.sin(theta), -sp.sin(alpha)*sp.sin(theta), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(alpha)*sp.cos(theta), -sp.sin(alpha)*sp.cos(theta), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1],
    ])

    return DH_mat

