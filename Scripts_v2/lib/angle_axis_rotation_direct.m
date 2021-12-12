function R = angle_axis_rotation_direct(r, theta)
% angle-axis rotation representation, DIRECT PROBLEM 
    I = eye(3);
    R = r*r' + (I-r*r')*cos(theta)+S(r)*sin(theta);
end

