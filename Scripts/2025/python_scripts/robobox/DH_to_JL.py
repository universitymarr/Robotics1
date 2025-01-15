import sympy as sp
from .DH_matrix import DH_matrix

def DH_to_JL(DHTABLE: list, variables: list) -> sp.Matrix:
    """
    Computes the Linear Jacobian matrix from the provided DH table
    
    @param DHTABLE: The Denavit-Hartenberg table
    @param variables: The variables list
    """
    N = len(DHTABLE)
    
    alpha, a, d, theta = sp.symbols('alpha a d theta')
    DH_mat = DH_matrix()

    A = []
    for i in range(N):
        A_i = DH_mat.subs({
            alpha: DHTABLE[i][0],
            a: DHTABLE[i][1],
            d: DHTABLE[i][2],
            theta: DHTABLE[i][3]
        })
        A.append(A_i)

    T = sp.eye(4)
    p0i = []

    for i in range(N):
        T = T @ A[i]
        p0i.append(T[0:3, 3]) 

    p0e = p0i[-1]

    JL = sp.simplify(sp.Matrix([p0e]).jacobian(variables))

    return JL