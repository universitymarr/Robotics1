function [angles] = inverse_kinematics_2R_planar(l1,l2,px,py,pos_neg)
    %Function that returns the value of the angles q1 and q2 
    %of a 2R planar robot
    %
    %parameters:
    %- l1= length of the first link
    %- l2= length of the second link
    %- px= x coordinate of the cartesian end effector position
    %- py= y coordinate of the cartesian end effector position
    %- pos_neg= a string that specifies wheter the solution comes
    %   from the positive root of the sine or the negative one
    %   ("pos" or "neg")
    %
    %output:
    %- q1= value of the angles of the first joint
    %- q2= value of the angle of the second joint
    
    %Find the cosine of the second angle
    c2 = (px^2 + py^2 - (l1^2 + l2^2)) / (2 * l1 * l2);

    %Fine the sin of the second angle depending on the sign chosen
    if strcmp(pos_neg, 'pos')
        s2 = sqrt(1 - c2^2);
    elseif strcmp(pos_neg, 'neg')
        s2 = -sqrt(1 - c2^2);
    else
        error('Invalid pos_neg argument. Use "pos" or "neg".');
    end
    
    %compute the value of q2
    q2 = atan2(s2, c2);
    
    %compute the value of q1
    q1 = atan2(py, px) - atan2(l2 * s2, l1 + l2 * c2);
    
    angles=[q1,q2];
end