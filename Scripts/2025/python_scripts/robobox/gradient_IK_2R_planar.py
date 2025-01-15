import sympy as sp
import numpy as np
from .compute_jacobian import compute_jacobian

def gradient_IK_2R_planar(pd: np.ndarray, q0: np.ndarray, l: np.ndarray, 
                          kmax: int=3, eps: float=1e-4, alpha: float=1e-4, verbose: int=1) -> tuple:
    """
    Computes the 2R planar robot Inverse Kinematics using the Gradient Descent method

    @param pd: The desired end-effector position
    @param q0: The initial joint angles values
    @param l: The joint lengths
    @param kmax: The maximum number of iterations
    @param eps: The tolerance for convergence
    @param alpha: The learning rate
    @param verbose: The control flag
    """
    l1, l2 = l
    theta1, theta2, l1_sym, l2_sym = sp.symbols('theta1 theta2 l1 l2')

    # 2R Forward Kinematics
    x = l1_sym * sp.cos(theta1) + l2_sym * sp.cos(theta1 + theta2)
    y = l1_sym * sp.sin(theta1) + l2_sym * sp.sin(theta1 + theta2)

    functions = [x, y]
    variables = [theta1, theta2]

    J_sym = compute_jacobian(functions, variables)
    q = np.copy(q0)

    q_lst = [q0]
    e_lst = []

    for k in range(kmax):
        if verbose:
            print(f"===== Iteration n°{k+1} =====")

        # Using numerical values of the Jacobia
        J_num = np.array([[J_sym[0, 0].subs({theta1: q[0], theta2: q[1], l1_sym: l1, l2_sym: l2}).evalf(),
                           J_sym[0, 1].subs({theta1: q[0], theta2: q[1], l1_sym: l1, l2_sym: l2}).evalf()],
                          [J_sym[1, 0].subs({theta1: q[0], theta2: q[1], l1_sym: l1, l2_sym: l2}).evalf(),
                           J_sym[1, 1].subs({theta1: q[0], theta2: q[1], l1_sym: l1, l2_sym: l2}).evalf()]], dtype=np.float64)

        x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
        y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
        f = np.array([x, y])

        e = pd - f
        error_norm = np.linalg.norm(e)
        e_lst.append(e)

        if error_norm <= eps:
            if verbose:
                print(f"Convergence reached in {k+1} steps")
                print(f"Computed joint angles for target position {pd}: (q0, q1) = {q}")
                print(f"Final error norm: {error_norm}")
            return q, error_norm, q_lst, e_lst

        q += alpha * np.dot(J_num.T, e) 
        q_lst.append(q)

        if verbose:
            print(f"(q0, q1) = {q}")
            print(f"Error norm: {error_norm}")

    if verbose:
        print(f"==== Max iterations reached ====")
        print(f"Computed joint angles for target position {pd}: (q0, q1) = {q}")
        print(f"Final error norm: {error_norm}")

    return q, error_norm, q_lst, e_lst
