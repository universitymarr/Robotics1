    function [theta, r] = get_theta_r(R)
% [theta, r] = get_theta_r(R) takes as inputs:
%   -R: A rotation matrix, tipycally a 3x3 matrix
% and outputs:
%   -theta: The radiants the rotation has been performed of
%   -r: The vector the rotation has been performed about
% Remember to use eval(symbolic function) to use this function
    
   
    
    risp = input("You can use either the positive or the negative sin(theta), this will correspond to 2 different solutions, which one you prefer? (pos, neg)\n", "s");
    
    if risp == "pos"
        sintheta= sqrt((R(1,2)-R(2,1))^2 + (R(1,3)-R(3,1))^2 + (R(2,3)-R(3,2))^2)/2;
        costheta= (R(1,1)+R(2,2)+R(3,3)-1)/2
        theta = atan2(sintheta,costheta);
        if theta == 0
            disp('Theta = 0, rotation axis is not defined, hence there is not a given solution');
            r = -1;
        else
            fprintf("Theta is %.2f (should be either +/- pi), hence we are in a singular case -> sin(theta) == 0\n", theta);
        
            r = (1/(2*sintheta))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2);];
        end
    else
        sintheta= -sqrt((R(1,2)-R(2,1))^2 + (R(1,3)-R(3,1))^2 + (R(2,3)-R(3,2))^2)/2;
        costheta= (R(1,1)+R(2,2)+R(3,3)-1)/2
        theta = atan2(sintheta,costheta);
        
        if theta == 0
            disp('Theta = 0, rotation axis is not defined, hence there is not a given solution');
            r = -1;
        else
            fprintf("Theta is %.2f (should be either +/- pi), hence we are in a singular case -> sin(theta) == 0\n", theta);
        
            r = (1/(2*sintheta))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2);];
        end
        
        
    end
        
    
  