function [axis, angle] = axis_angle_rotation_inverse(R, pos_or_neg)
    % Function that generates an angle and an axis given a rotation matrix
    %
    % Parameters:
    % - R: rotation matrix
    % - pos_or_neg: string "pos"/"neg" that specifies which solution to return
    %
    % Returns:
    % - axis: axis of rotation
    % - angle: angle of rotation
    
    % Check if the input matrix is a valid rotation matrix
    % A valid rotation matrix should be 3x3, real, have determinant 1, and be orthogonal (R * R' = I)
    if ~isequal(size(R), [3, 3]) && isreal(R) && det(R) > 0 && isequal(R * R', eye(3))
        error('Input is not a valid rotation matrix.');
    end
    
    % Extract angle from the rotation matrix
    % The trace of a rotation matrix R is related to the rotation angle θ by:
    % trace(R) = 1 + 2cos(θ)
    % Solving for θ gives: θ = acos((trace(R) - 1) / 2)
    angle = acos((trace(R) - 1) / 2);
    
    % Extract axis from the rotation matrix
    % For a rotation matrix R, (R - R') / (2 * sin(θ)) gives a skew-symmetric matrix
    % This skew-symmetric matrix represents the cross-product matrix of the rotation axis
    axisSkewSymmetric = (R - R') / (2 * sin(angle));

    % Extract the components of the axis from the skew-symmetric matrix
    % In a skew-symmetric matrix S = [0 -z y; z 0 -x; -y x 0],
    % the axis components [x; y; z] can be read off the matrix
    axis = [axisSkewSymmetric(3, 2); axisSkewSymmetric(1, 3); axisSkewSymmetric(2, 1)];

    % Ensure positive or negative solution based on the input variable
    % The angle-axis representation has two equivalent forms: (r, θ) and (-r, -θ)
    % This step allows the user to choose which form they want
    if pos_or_neg=="pos"
        angle = abs(angle);
    else
        angle = -abs(angle);
    end
end