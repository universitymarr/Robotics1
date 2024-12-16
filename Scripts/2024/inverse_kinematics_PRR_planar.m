function joint_values = inverse_kinematics_PRR_planar(L, x, y, alpha, pos_neg)
    % Function that performs the inverse kinematic of an RPR robot
    % Parameters:
    % - L: Lenght of the first link
    % - x: x coordinate of end effector position
    % - y: y coordinate of end effector position
    % - alpha: angle of the end effector (most of the time given by q2 + q3)
    % - pos_neg: a string that indicates what solution you want "pos" or "neg"

    % Calculate the joint values
    if pos_neg == "pos"
        q1 = x - L * cos(alpha) + sqrt(L^2 * cos(alpha)^2 + 2 * L * sin(alpha) * y - y^2);
    else
        q1 = x - L * cos(alpha) - sqrt(L^2 * cos(alpha)^2 + 2 * L * sin(alpha) * y - y^2);
    end

    q2 = atan2((y/L) - sin(alpha), ((x-q1)/L) - cos(alpha));
    q3 = alpha - q2;

    disp("Joint values:")
    disp("- q1:");
    disp(q1);
    disp("- q2:");
    disp(q2);
    disp("- q3:");
    disp(q3);

    joint_values = [q1, q2, q3];
end
