import sympy as sp
from .DH_matrix import DH_matrix  

def DH_to_JA(DHTABLE: list, prismatic_joints: list=[]) -> sp.Matrix:
    """
    Computes the Angular Jacobian matrix from the provided DH table
    
    @param DHTABLE: The Denavit-Hartenberg table
    @param prismatic_joints: The list of prismatic joint indexes
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
    R0i = []

    for i in range(N):
        T = T @ A[i]
        R0i.append(T[0:3, 0:3])

    JA = sp.zeros(3, N)

    z = sp.Matrix([0, 0, 1])
    JA[:, 0] = z

    for i in range(1, N):
        R = R0i[i-1]   # Rotation matrix for joint i-1
        zi = R @ z     # Transformed z-axis
        JA[:, i] = zi

    # Remove the prismatic joints
    if len(prismatic_joints) > 0:
        for prismatic_index in prismatic_joints:
            if prismatic_index != N:
                JA[:, prismatic_index] = sp.zeros(3, 1)

    return JA