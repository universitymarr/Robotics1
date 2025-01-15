import numpy as np

def axisangle_rotation(r: np.ndarray, theta: float) -> np.ndarray:
    """
    Computes the axis/angle rotation for a 3D matrix
    
    @param r: The unit vector
    @param theta: the rotation angle
    """
    I = np.eye(3)

    # Outer product
    rrT = np.outer(r,r)

    # Skew symmetric matrix
    S = np.array([[ 0, -r[2], -r[1]],
                  [r[2], 0, -r[0]],
                  [-r[1], r[0], 0]])
    
    # Rodrigues' rotation formula 
    R = rrT + (I - rrT)*np.cos(theta) + S*np.sin(theta)

    return R
