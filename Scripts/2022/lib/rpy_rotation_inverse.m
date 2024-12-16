function [phi, theta, psi] = rpy_rotation_inverse(sequence, R, pos_or_neg_solution)
% [phi, theta, psi] = rpy_rotation_inverse(sequence, R) takes as inputs:
%   -sequence: a string which specifies how the RPY-rotation has been computed, e.g. "xyx"
%   -R: the rotation to be decomposed, should be a 3x3 matrix
% and outputs:
%   -phi: the radiants of the first rotation
%   -theta: the radiants of the second rotation
%   -psi: the radiants of the third rotation
% Upon execution the function will ask for an input which will determine
% which solution must be displayed (there is always a pair of solutions)
%
% RPY rotations work about fixed-axes

    sequence = char(sequence);
    %[phi, theta, psi] = euler_rotation_inverse(flip(sequence), R, pos_or_neg_solution);
    [psi, theta, phi] = euler_rotation_inverse(flip(sequence), R, pos_or_neg_solution);
