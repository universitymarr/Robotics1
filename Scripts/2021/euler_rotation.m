    function R = euler_rotation(sequence, angles)
% R = euler_rotation(sequence, angles) takes as inputs:
%   -sequence: a string which specifies the axes along which rotation
%              occurs
%   -angles: The radiants (or symbolics) of the three rotations
% and outputs:
%   -R: The desired rotation
% Euler rotations work about moving-axes
    
    if strlength(sequence) ~= 3
        disp("Sequence not valid, must be of lenght three.")
        return;
    end
    
    sequence = lower(char(sequence));
    if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
        disp("Two consecutive rotation along the same axis are not valid.")
        return
    end
    
    R = elem_rot_mat(sequence(1), angles(1)) * elem_rot_mat(sequence(2), angles(2)) * elem_rot_mat(sequence(3), angles(3));

end