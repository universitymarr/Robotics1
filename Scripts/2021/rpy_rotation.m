function R = rpy_rotation(sequence, angles)
% R = rpy_rotation(sequence, angles) takes as inputs:
%   -sequence: a string which specifies which elementary rotation matrices
%              must be multiplied together to obtain the desired rotation
%   -angles: the radiants of the three rotation angles
% and outputs:
%   -R: The desired rotation
% RPY rotations work about fixed-axes

    sequence = char(sequence);
    R = euler_rotation(flip(sequence), flip(angles));
end