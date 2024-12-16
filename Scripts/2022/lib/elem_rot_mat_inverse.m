function alpha = elem_rot_mat_inverse(axis, rotation_matrix)
    switch axis
        case {"x", "X"}
%             R = [1       0        0;
%                  0    cos(s)   -sin(s);
%                  0    sin(s)    cos(s)];
             
            sin_alpha = rotation_matrix(3, 2);
            cos_alpha = rotation_matrix(3, 3);
            alpha = atan2(sin_alpha, cos_alpha);
            
        case {"y", "Y"}            
%             R = [cos(s)     0   sin(s);
%                     0	    1     0;
%                 -sin(s)     0   cos(s)];

            sin_alpha = rotation_matrix(1, 3);
            cos_alpha = rotation_matrix(1, 1);
            alpha = atan2(sin_alpha, cos_alpha);
        
        case {"z", "Z"}
%             R = [cos(s)  -sin(s)    0;
%                  sin(s)   cos(s)    0;
%                    0        0       1];

            sin_alpha = rotation_matrix(2, 1);
            cos_alpha = rotation_matrix(1, 1);
            alpha = atan2(sin_alpha, cos_alpha);
        otherwise
            disp("First Parameter should be either 'x', 'y', 'z' or any of those capitalized")
    end

end
