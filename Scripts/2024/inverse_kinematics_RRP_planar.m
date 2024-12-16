function [q1, q2, q3] = inverse_kinematics_RRP_planar(L1, x, y, phi, pos_neg)
    % Function that performs the inverse kinematics of an RRP robot
    %
    % Parameters:
    % - L1: length of the first link
    % - x: x-coordinate of the end-effector position
    % - y: y-coordinate of the end-effector position
    % - phi: sum of q1 and q2 (q1 + q2)

    % Calculate q3
    A = 1;
    B = -2 * (x * cos(phi) + y * sin(phi));
    C = x^2 + y^2 - L1^2;
    discriminant = B^2 - 4 * A * C;

    if discriminant > 0
        % Choose the positive or negative solution based on the sign
        if pos_neg == "pos"
            q3 = (-B + sqrt(discriminant)) / (2 * A);           
        else
            q3 = (-B - sqrt(discriminant)) / (2 * A);
        end
    elseif discriminant == 0
        q3 = -B / (2 * A);
    else
        disp('No real solution exists for q3.');
        q1 = NaN;
        q2 = NaN;
        q3 = NaN;
        return
    end

    % Calculate q1 and q2
    sin_q1 = (y - q3 * sin(phi)) / L1;
    cos_q1 = (x - q3 * cos(phi)) / L1;
    q1 = atan2(sin_q1, cos_q1);
    q2 = phi - q1;

    disp('Joint values:');
    disp(['- q1: ', num2str(q1)]);
    disp(['- q2: ', num2str(q2)]);
    disp(['- q3: ', num2str(q3)]);
end