function omega = rpy_rotation_omega(sequence, angles)
% omega = rpy_rotation_omega(sequence, angular_velocities) takes as inputs:
%   -sequence: The sequence of rotations performed, e.g. "xyx"
%   -angles: The radiants (or symbolics) of the three rotations
% and outputs:
%   -omega: The total angular velocity
% Need to re-define both the function and the comment
% Remember to express the angles in terms of the symbol t

    sequence = char(sequence);
    omega = euler_rotation_omega(flip(sequence), flip(angles));
end