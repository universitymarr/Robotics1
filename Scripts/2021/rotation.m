function rot = rotation(theta, r)
% rot = rotation(theta, r) takes as inputs:
%   -theta: the radiants we want to perform the rotation of
%   -r: the vector we want to perform the rotation about
% and outputs:
%   -R: the desired rotation

    c = (1 - cos(theta));
    rot = [
        r(1)^2*c + cos(theta)            r(1)*r(2)*c - r(3)*sin(theta)  r(1)*r(3)*c + r(2)*sin(theta);
        r(1)*r(2)*c + r(3)*sin(theta)    r(2)^2*c + cos(theta)          r(2)*r(3)*c - r(1)*sin(theta);
        r(1)*r(3)*c - r(2)*sin(theta)    r(2)*r(3)*c + r(1)*sin(theta)  r(3)^2*c + cos(theta)
    ];
          
end