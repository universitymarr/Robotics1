function Rmatrix = ElementaryRotationM(axis, angle)
    % Check if the axis is valid
    if ~(axis == 'X' || axis == 'Y' || axis == 'Z')
        error('Invalid axis. Use ''X'', ''Y'', or ''Z''.');
    end

    % Initialize symbolic variables
    syms theta

    % Calculate the rotation matrix based on the chosen axis
    if axis == 'X'
        Rmatrix = [1, 0, 0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
    elseif axis == 'Y'
        Rmatrix = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    elseif axis == 'Z'
        Rmatrix = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    else
        error('Invalid axis. Use ''X'', ''Y'', or ''Z''.');
    end

    % Substitute the symbolic angle 'angle' for 'theta'
    Rmatrix = subs(Rmatrix, theta, angle);
end
