function [phi, theta, psi] = euler_rotation_inverse(sequence, R)
% [phi, theta, psi] = euler_rotation_inverse(sequence, R) takes as inputs:
%   -sequence: a string which specifies how the euler-rotation has been computed, e.g. "xyx"
%   -R: the rotation to be decomposed, should be a 3x3 matrix
% and outputs:
%   -phi: the radiants of the first rotation
%   -theta: the radiants of the second rotation
%   -psi: the radiants of the third rotation
% Upon execution the function will ask for an input which will determine
% which solution must be displayed (there is always a pair of solutions)
%
% Euler rotations work about moving-axes
    
    if isa(R, 'sym')
        disp("R must by non-symbolic matrix")
        phi = -1; theta = -1; psi = -1;
        return
    end
    
    if strlength(sequence) ~= 3
        disp("Sequence not valid, must be of length three.")
        return;
    end
    
    if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
        disp("Two consecutive rotation along the same axis are not valid.")
        return
    end
    
    risp = input("There is always a pair of solutions to the inverse euler problem. Do you want to use the positive or negative sin(theta)? (pos, neg)\n", "s");
    cond = risp == "pos";
    
    switch lower(sequence)
        case "xyx"
            theta = atan2(sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*cond + atan2(-sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 2)/sin(theta), R(1, 3)/sin(theta));
            phi = atan2(R(2, 1)/sin(theta), -R(3, 1)/sin(theta));
            
        case "xyz"
            theta = atan2(R(1, 3), sqrt(R(1, 1)^2 + R(1, 2)^2))*cond + atan2(R(1, 3), -sqrt(R(1, 1)^2 + R(1, 2)^2))*(1-cond);
            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(1, 2)/cos(theta), R(1, 1)/cos(theta));
            phi = atan2(-R(2, 3)/cos(theta), R(3, 3)/cos(theta));
            
        case "xzx"
            theta = atan2(sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*cond + atan2(-sqrt(R(1, 2)^2 + R(1, 3)^2), R(1, 1))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 3)/sin(theta), -R(1, 2)/sin(theta));
            phi = atan2(R(3, 1)/sin(theta), R(2, 1)/sin(theta));
            
        case "xzy"
            theta = atan2(-R(1, 2), sqrt(R(1, 1)^2 + R(1, 3)^2))*cond + atan2(-R(1, 1), -sqrt(R(1, 3)^2 + R(2, 1)^2))*(1-cond);
            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(1, 3)/cos(theta), R(1, 1)/cos(theta));
            phi = atan2(R(3, 2)/cos(theta), R(2, 2)/cos(theta));
            
        case "yxy"
            theta = atan2(sqrt(R(2, 3)^2 + R(2, 1)^2), R(2, 2))*cond + atan2(-sqrt(R(2, 3)^2 + R(2, 1)^2), R(2, 2))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 1)/sin(theta), -R(2, 3)/sin(theta));
            phi = atan2(R(1, 2)/sin(theta), R(3, 2)/sin(theta));
            
        case "yxz"
            theta = atan2(-R(2, 3), sqrt(R(2, 2)^2 + R(2, 1)^2))*cond + atan2(-R(2, 3), -sqrt(R(2, 2)^2 + R(2, 1)^2))*(1-cond);
            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 1)/cos(theta), R(2, 2)/cos(theta));
            phi = atan2(R(1, 3)/cos(theta), R(3, 3)/cos(theta));
            
        case "yzx"
            theta = atan2(R(2, 1), sqrt(R(2, 2)^2 + R(2, 3)^2))*cond + atan2(R(2, 1), -sqrt(R(2, 2)^2 + R(2, 3)^2))*(1-cond);

            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(2, 3)/cos(theta), R(2, 2)/cos(theta));
            phi = atan2(-R(3, 1)/cos(theta), R(1, 1)/cos(theta));
            
        case "yzy"
            theta = atan2(sqrt(R(2, 1)^2 + R(2,3)^2), R(2, 2))*cond + atan2(-sqrt(R(2, 1)^2 + R(2,3)^2), R(2, 2))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(2, 3)/sin(theta), R(2, 1)/sin(theta));
            phi = atan2(R(3, 2)/sin(theta), -R(1, 2)/sin(theta));
            
        case "zxy"
            theta = atan2(R(3, 2), sqrt(R(3, 1)^2 + R(3,3)^2))*cond + atan2(R(3, 2), -sqrt(R(3, 1)^2 + R(3,3)^2))*(1-cond);
            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(-R(3, 1)/cos(theta), R(3, 3)/cos(theta));
            phi = atan2(-R(1, 2)/cos(theta), R(2, 1)/cos(theta));
            
        case "zxz"
            theta = atan2(sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3))*cond + atan2(-sqrt(R(1, 3)^2 + R(2, 3)^2), R(3, 3))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(3, 1)/sin(theta), R(3, 2)/sin(theta));
            phi = atan2(R(1, 3)/sin(theta), -R(2, 3)/sin(theta));
            
        case "zyx"
            theta = atan2(-R(3, 1), sqrt(R(3, 2)^2+R(3, 3)^2))*cond + atan2(-R(3, 1), -sqrt(R(3, 2)^2+R(3, 3)^2))*(1-cond);
            if (abs(cos(theta)) <= 1e-6)
                disp('Singular case: cos(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(3, 2)/cos(theta), R(3, 3)/cos(theta));
            phi = atan2(R(2, 1)/cos(theta), R(1, 1)/cos(theta));
            
        case "zyz"
            theta = atan2(sqrt(R(3, 1)^2 + R(3, 2)^2), R(3, 3))*cond + atan2(-sqrt(R(3, 1)^2 + R(3, 2)^2), R(3, 3))*(1-cond);
            if (abs(sin(theta)) <= 1e-6)
                disp('Singular case: sin(theta) == 0 or very close to 0.')
                return
            end
            
            psi = atan2(R(3, 2)/sin(theta), -R(3, 1)/sin(theta));
            phi = atan2(R(2, 3)/sin(theta), R(1, 3)/sin(theta));
            
        otherwise
            disp("Invalid sequence")
    end
    
end