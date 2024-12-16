function R = euler_rotation(sequence, angles)
    % R = euler_rotation(sequence, angles) takes as inputs:
    %   -sequence: a string which specifies the axes along which rotation
    %              occurs
    %   -angles: The radiants (or symbolics) of the three rotations
    % and outputs:
    %   -R: The desired rotation
    % Euler rotations work about moving-axes
        if strlength(sequence) ~= 3
            disp("Sequence not valid, must be of length three.")
            return;
        end
        
        sequence = lower(char(sequence));
        if (sequence(2) == sequence(1) || sequence(2) == sequence(3))
            disp("Two consecutive rotations along the same axis are not valid.")
            return
        end
        
        % Convert sequence to a cell array of uppercase characters
        sequence_array = upper(cellstr(sequence'));
        
        R = ElementaryRotationM(sequence_array{1}, angles(1)) * ...
            ElementaryRotationM(sequence_array{2}, angles(2)) * ...
            ElementaryRotationM(sequence_array{3}, angles(3));
    end