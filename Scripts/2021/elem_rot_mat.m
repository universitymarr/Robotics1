function R = elem_rot_mat(axis, s)
% R = elem_rot_mat(axis, angle) takes as inputs:
%   -axis: The axis we should perform the rotation about, ("x", "y", "z")
%   -angle: The radiants (or a symbolic) we should perform the rotation of
% and outputs:
%   -R: The desired elementary rotation

    switch axis
        case {"x", "X"}
            R = [1       0        0;
                 0    cos(s)   -sin(s);
                 0    sin(s)    cos(s)];
        case {"y", "Y"}
            R = [cos(s)     0   sin(s);
                   0	    1     0;
                -sin(s)     0   cos(s)];
        case {"z", "Z"}
            R = [cos(s)  -sin(s)    0;
                 sin(s)   cos(s)    0;
                   0        0       1];
        otherwise
            disp("First Parameter should be either 'x', 'y', 'z' or any of those capitalized")
    end
    
end