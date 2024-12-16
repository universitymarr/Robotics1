function [angles] = inverse_kinematics_3R_spatial(L, M, N, p, pos_neg)
    %Function that returns the value of the angles q1 and q2 
    %of a 2R planar robot
    %
    %parameters:
    %- l1= length of the first link
    %- l2= length of the second link
    %- px= x coordinate of the cartesian end effector position
    %- py= y coordinate of the cartesian end effector position
    %- pos_neg= a string that specifies wheter the solution comes
    %   from the positive root of the sine or the negative one
    %   ("pos" or "neg")
    %
    %output:
    %- q1= value of the angles of the first joint
    %- q2= value of the angle of the second joint
   
    s3 = (p(3) - M)/N; % Compute the value of sin(q3)
    if strcmp(pos_neg, 'pos')
        c3 = sqrt(1 - s3^2); % Compute the value of cos(q3)
    elseif strcmp(pos_neg, 'neg')
        c3 = -sqrt(1 - s3^2); % Compute the value of cos(q3)
    else
        error('Invalid pos_neg argument. Use "pos" or "neg".');
    end
    
    q3 = atan2(s3, c3); % Compute the value of q3

    % Now we want to find q1 from the first 2 equations in p
    l1 = L;
    l2 = N * cos(q3);

    c2 = (p(1)^2 + p(2)^2 - (l1^2 - l2^2)) / 2 * l1 * l2;

    if strcmp(pos_neg, 'pos')
        s2 = sqrt(1 - c2^2);
    elseif strcmp(pos_neg, 'neg')
        s2 = -sqrt(1 - c2^2);
    else
        error('Invalid pos_neg argument. Use "pos" or "neg".');
    end
    
    q2 = atan2(s2, c2);

    % Now, in order to find s1 and s2, we can set up a system of equations in the form:
    % Ax = b where x = [c1; s1], b = [p(1); p(2)] and A = [...] and solving its Determinant
    det_A = L^2 + N^2 * cos(q3)^2 - 2 * L * N * cos(q3) * cos(q2);

    % Now find s1 and c1, we must:
    % - Expand the sin/cos sum in p(1) and p(2)
    % - Simplify the notation:
    %   - a = L + N * cos(q2) * cos(q3)
    %   - b = N * sin(q2) * cos(q3)
    % So that the system of equations becomes:
    % a * c1 - b * s1 = p(1)
    % b * c1 + a * s1 = p(2)
    %
    % In order to solve for s1 and c1, we can use the Cramer's rule
    % s1 = ...
    % c1 = ...
    s1 = p(2) * (l1 + l2*c2) - p(1) * l2 * s2/det_A;
    c1 = p(1) * (l1 + l2*c2) + p(2) * l2 * s2/det_A;

    % Compute the value of q1
    q1 = atan2(s1, c1);
    
    angles=[q1, q2, q3];
end