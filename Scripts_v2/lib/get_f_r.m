function f_r = get_f_r(T)
% f_r = get_f_r(T) takes as inputs:
%   -T: symbolic function of a homougeneous rototranslation matrix
%   -alpha_z: the fourth coordinate of the f_r mapping
%   -q: array of symbolic 
% and outputs:
%   -f_r: mapping from joint space to cartesian space

    if ~isa(T, 'sym')
        disp("We need a sym homogeneous matrix.");
        f_r = -1;
        return
    end
    
    alpha_z = sum(symvar(T(1:3, 1:3)));
    f_r = [T(1, 4); T(2, 4); T(3, 4); alpha_z];

end

% how to obtain f_r, (i.e. r)
%{
    From DH table:
        A0_2 = A0_1 * A1_2 //mapping from frame 2 to frame 0

    So, to obtain the origin of frame 2 in frame 0:
        O2_2 = [0 0 0] % 3D space
            -> O2_2_H = [0 0 0 1] //homog. transf
        
        r = O0_2 = [rx ry rz 1]' = A0_2*O2_2_H 
%}
