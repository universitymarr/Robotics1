function omega = euler_rotation_omega(sequence, angles)
% omega = euler_rotation_omega(sequence, angular_velocities) takes as input
%   -sequence: The sequence of rotations performed, e.g. "xyx"
%   -angles: The radiants (or symbolics) of the three rotations
% and outputs:
%   -omega: The total angular velocity
% Need to re-define both the function and the comment
% Remember to express the angles in terms of the symbol t

    if strlength(sequence) ~= 3
        disp("Sequence not valid, must be of length three.")
        omega = -1;
        return;
    end
    
    sequence = lower(char(sequence));
    if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
        disp("Two consecutive rotation along the same axis are not valid.")
        omega = -1;
        return
    end
    
    first_rot = elem_rot_mat(sequence(1), angles(1));
    second_rot = first_rot * elem_rot_mat(sequence(2), angles(2));
    
    switch sequence(1)
        case "x"
            z = [1 0 0].';
        case "y"
            z = [0 1 0].';
        case "z"
            z = [0 0 1].';
    end
    
    switch sequence(2)
        case "x"
            y = first_rot(:, 1);
        case "y"
            y = first_rot(:, 2);
        case "z"
            y = first_rot(:, 3);
    end
    
    switch sequence(3)
        case "x"
            x = second_rot(:, 1);
        case "y"
            x = second_rot(:, 2);
        case "z"
            x = second_rot(:, 3);
    end
    
    [x y z]
    omega = [x y z] * reshape(angles, [3, 1]);
    
end