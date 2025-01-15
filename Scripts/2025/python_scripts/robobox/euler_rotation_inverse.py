import numpy as np

def euler_rotation_inverse(sequence: str, R: np.ndarray, solution_sign: str) -> list:
    """
    Computes the Inverse Euler rotation
    
    @param: sequence: The rotation sequence (e.g., "xyx", "xyz", etc.)
    @param: R: The 3x3 rotation matrix to be decomposed
    @param: solution_sign: Determines which solution to use ("pos" or "neg")
    """
    if R.shape != (3, 3):
        raise ValueError("R must be a 3x3 matrix.")
    
    if len(sequence) != 3:
        raise ValueError("Sequence must be of length 3.")
    
    if sequence[1] == sequence[0] or sequence[1] == sequence[2]:
        raise ValueError("Two consecutive rotations along the same axis are not valid.")
    
    cond = solution_sign == "pos"
    
    if sequence.lower() == "xyx":
        theta = np.arctan2(np.sqrt(R[0, 1]**2 + R[0, 2]**2), R[0, 0])*cond + \
                np.arctan2(-np.sqrt(R[0, 1]**2 + R[0, 2]**2), R[0, 0])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[0, 1]/np.sin(theta), R[0, 2]/np.sin(theta))
        phi = np.arctan2(R[1, 0]/np.sin(theta), -R[2, 0]/np.sin(theta))
    
    elif sequence.lower() == "xyz":
        theta = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))*cond + \
                 np.arctan2(R[0, 2], -np.sqrt(R[0, 0]**2 + R[0, 1]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(-R[0, 1]/np.cos(theta), R[0, 0]/np.cos(theta))
        phi = np.arctan2(-R[1, 2]/np.cos(theta), R[2, 2]/np.cos(theta))
    
    elif sequence.lower() == "xzx":
        theta = np.arctan2(np.sqrt(R[0, 1]**2 + R[0, 2]**2), R[0, 0])*cond + \
                 np.arctan2(-np.sqrt(R[0, 1]**2 + R[0, 2]**2), R[0, 0])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[0, 2] / np.sin(theta), -R[0, 1] / np.sin(theta))
        phi = np.arctan2(R[2, 0] / np.sin(theta), R[1, 0] / np.sin(theta))
    
    elif sequence.lower() == "xzy":
        theta = np.arctan2(-R[0, 1], np.sqrt(R[0, 0]**2 + R[0, 2]**2))*cond + \
                 np.arctan2(-R[0, 0], -np.sqrt(R[0, 2]**2 + R[1, 0]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) ==0 or very close to 0.")
            return
        psi = np.arctan2(R[0, 2] / np.cos(theta), R[0, 0] / np.cos(theta))
        phi = np.arctan2(R[2, 1] / np.cos(theta), R[1, 1] / np.cos(theta))
    
    elif sequence.lower() == "yxy":
        theta = np.arctan2(np.sqrt(R[1, 2]**2 + R[1, 0]**2), R[1, 1])*cond + \
                 np.arctan2(-np.sqrt(R[1, 2]**2 + R[1, 0]**2), R[1, 1])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[1, 0] / np.sin(theta), -R[1, 2] / np.sin(theta))
        phi = np.arctan2(R[0, 1] / np.sin(theta), R[2, 1] / np.sin(theta))
    
    elif sequence.lower() == "yxz":
        theta = np.arctan2(-R[1, 2], np.sqrt(R[1, 1]**2 + R[1, 0]**2))*cond + \
                 np.arctan2(-R[1, 2], -np.sqrt(R[1, 1]**2 + R[1, 0]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) ==0 or very close to 0.")
            return
        psi = np.arctan2(R[1, 0] / np.cos(theta), R[1, 1] / np.cos(theta))
        phi = np.arctan2(R[0, 2] / np.cos(theta), R[2, 2] / np.cos(theta))
    
    elif sequence.lower() == "yzx":
        theta = np.arctan2(R[1, 0], np.sqrt(R[1, 1]**2 + R[1, 2]**2))*cond + \
                 np.arctan2(R[1, 0], -np.sqrt(R[1, 1]**2 + R[1, 2]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(-R[1, 2] / np.cos(theta), R[1, 1] / np.cos(theta))
        phi = np.arctan2(-R[2, 0] / np.cos(theta), R[0, 0] / np.cos(theta))
    
    elif sequence.lower() == "yzy":
        theta = np.arctan2(np.sqrt(R[1, 0]**2 + R[1, 2]**2), R[1, 1])*cond + \
                 np.arctan2(-np.sqrt(R[1, 0]**2 + R[1, 2]**2), R[1, 1])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[1, 2] / np.sin(theta), -R[1, 0] / np.sin(theta))
        phi = np.arctan2(R[2, 1] / np.sin(theta), -R[0, 1] / np.sin(theta))
    
    elif sequence.lower() == "zxy":
        theta = np.arctan2(R[2, 1], np.sqrt(R[2, 0]**2 + R[2, 2]**2))*cond + \
                 np.arctan2(R[2, 1], -np.sqrt(R[2, 0]**2 + R[2, 2]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(-R[2, 0] / np.cos(theta), R[2, 2] / np.cos(theta))
        phi = np.arctan2(-R[0, 1] / np.cos(theta), R[1, 0] / np.cos(theta))
    
    elif sequence.lower() == "zxz":
        theta = np.arctan2(np.sqrt(R[0, 2]**2 + R[1, 2]**2), R[2, 2])*cond + \
                 np.arctan2(-np.sqrt(R[0, 2]**2 + R[1, 2]**2), R[2, 2])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[2, 0] / np.sin(theta), R[2, 1] / np.sin(theta))
        phi = np.arctan2(R[0, 2] / np.sin(theta), -R[1, 2] / np.sin(theta))
    
    elif sequence.lower() == "zyx":
        theta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))*cond + \
                 np.arctan2(-R[2, 0], -np.sqrt(R[2, 1]**2 + R[2, 2]**2))*(1-cond)
        if np.abs(np.cos(theta)) <= 1e-6:
            print("Singular case: cos(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[2, 1] / np.cos(theta), R[2, 2] / np.cos(theta))
        phi = np.arctan2(R[1, 0] / np.cos(theta), R[0, 0] / np.cos(theta))
    
    elif sequence.lower() == "zyz":
        theta = np.arctan2(np.sqrt(R[2, 0]**2 + R[2, 1]**2), R[2, 2])*cond + \
                 np.arctan2(-np.sqrt(R[2, 0]**2 + R[2, 1]**2), R[2, 2])*(1-cond)
        if np.abs(np.sin(theta)) <= 1e-6:
            print("Singular case: sin(theta) = 0 or very close to 0.")
            return
        psi = np.arctan2(R[2, 1] / np.sin(theta), -R[2, 0] / np.sin(theta))
        phi = np.arctan2(R[1, 2] / np.sin(theta), R[0, 2] / np.sin(theta))
    else:
        raise ValueError("Invalid sequence.")
    
    return phi, theta, psi
