import numpy as np

def euler_rotation (sequence: str, angles: list) -> np.ndarray:
    """
    Computes the rotation matrix of Euler angles

    @param sequence: The rotation sequence (e.g., 'xyz', 'yxy')
    @param angles: The list of rotations angles
    """
    # Assertions
    if len(sequence) != 3:
        raise ValueError("Sequence is not valid, must be of size 3.")
    
    sequence = sequence.lower()
    if sequence[1] == sequence[0] or sequence[1] == sequence[2]:
        raise ValueError("Two consecutive rotations along the same axis is incorrect.")
    
    def elementary_rotation(axis: str, theta: float) -> np.ndarray:
        """
        Computes the elementary rotation axis along a given axis

        @param axis: The string of the provided axis (e.g., 'x', 'y', 'z')
        @param: The angle value
        """
        c, s = np.cos(theta), np.sin(theta)
        if axis == 'x':
            return np.array([[1, 0, 0],
                             [0, c, -s],
                             [0, s, c]])
        elif axis == 'y':
            return np.array([[c, 0, s],
                             [0, 1, 0],
                             [-s, 0, c]])
        elif axis == 'z':
            return np.array([[c, -s, 0],
                             [s, c, 0],
                             [0, 0, 1]])
        else:
            raise ValueError(f"Invalid axis: {axis}. Must be 'x', 'y', or 'z'.")
    
    # Computing of the rotation matrix
    R = np.eye(3)
    for axis, angle in zip(sequence, angles):
        R = R @ elementary_rotation(axis, angle)

    return R