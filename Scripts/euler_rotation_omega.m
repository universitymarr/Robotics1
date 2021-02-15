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
    
    if isa(angles, 'sym')
    %the array of angles should be in the form of [alpha beta gamma]
        dangles = [];
        for idx=1:length(angles)
            t= strcat('d', char(angles(idx)));
            dangles=[dangles, sym(t)];
        end
        
        fprintf("The orientation of a rigid body using the %s sequence of Euler angles\nis given by the rotation matrix\n", sequence);
        third_rot = second_rot * elem_rot_mat(sequence(3), angles(3));
        R = simplify(third_rot)
        
        fprintf("The angular velocity omega of the body can be obtained from the formula\nS(omega) = Rdot*RT, where S is a skew-symmetric matrix.\n");
        fprintf("So we have\n");
        Rdot=diff(R,angles(1))*dangles(1)+diff(R,angles(2))*dangles(2)+diff(R,angles(3))*dangles(3)
        
        S_omega=simplify(Rdot*transpose(R))
        
        fprintf("The linear mapping omega=T*phiDot is then extracted\nfrom the elements of the S matrix\n");
        omega=[S_omega(3,2);S_omega(1,3);S_omega(2,1)]
        
        fprintf("Another way to do it is to directly consider the three contributions on omega of the rotations\n");
    end
    
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
    
    omegaMatrix = [x y z]
    fprintf("Where this matrix is multiplied by the derivative of the angles [first second third]\n");
    fprintf("being each contribution to omega a vector itself, the order in the sum is irrelevant\n");
    
    %omega = [x y z] * reshape(angles, [3, 1]);
    
end
