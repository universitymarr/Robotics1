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

